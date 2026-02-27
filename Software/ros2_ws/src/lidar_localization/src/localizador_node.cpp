#include "localizador_node.hpp"

LocalizadorNode::LocalizadorNode() : Node("localizador_node") {
    // 1. Suscribirse al LiDAR
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
                "/scan", 
                10, 
                [this](sensor_msgs::msg::LaserScan::SharedPtr msg)
                    {scan_callback(msg);}
                );

    // 2. Publicar nuestra posición (X, Y)
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/robot_pose", 10);

    // 3. Publicar puntos que son obstáculos (para verlos en RViz)
    obstaculos_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/obstaculos", 10);
}

void LocalizadorNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
    // Como cuando miras un vídeo de un extranjero en youtube:
    // El tio habla otro idioma (coord polares [LaserScan]), pero tu solo hablas español (coord cartesianas [PointCloud2]). 
    // Entonces usas la traduccion automatica (projectLaser de laser_geometry::LaserProjection) para que te lo traduzca
    // De por si no viene activado la intensidad. El -1.0 es para que te de todo. Si pones 2 te da los valores a partir de 2m.
    sensor_msgs::msg::PointCloud2 cloud_msg;
    projector_.projectLaser(*scan_msg, cloud_msg, -1.0, laser_geometry::channel_option::Intensity); // Puntero para evitar copia de todos los puntos y su lentitud
    
    // cloud es el nombre de la variable
    // pcl::PointCloud -> contendedor
    // pcl::PointXYZ -> contenido
    // new pcl::PointCloud<pcl::PointXYZ> reserva de memoria
    // new reserva la memoria que pidas y te da un puntero (llave a este)[se guarda e cloud]. si lo pierdes (la variable muere) se queda la memoria ocupando sitio no se borra. 
    // ::Ptr crea un "puntero compartido". Mira cuantos usan la llave que le ha dado new, si es 0 borra el contenido. (Evita que se quede memoria vieja por ahi ocupando)
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

    // Como cuando miras un vídeo de un latino explicando algo.
    // Hablais los dos español (PointCloud2) pero cada uno a su manera, haciendo que no os entendais en la mitad de lo que decis.
    // Por eso en el traductor hay la opcion de traducir español España (pcl::PointCloud) y español Latino (sensor_msgs::msg::PointCloud2). 
    // Pues lo mismo pasas (pcl::fromROSMsg) de español Latino (cloud_msg) a español España (cloud) para entienderlo 
    pcl::fromROSMsg(cloud_msg, *cloud);

    procesar_beacons(cloud);
    detectar_obstaculos(cloud);
}

void LocalizadorNode::detectar_obstaculos(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
bool emergencia_freno = false;
    float ancho_robot_medio = 0.20; 
    float distancia_seguridad = 0.60; 

    // En cloud no estan por orden, entonces KdTree hace un mapa de los indice donde los relaciona i ordena. Para luego poder buscar al rededor
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>); 
    tree->setInputCloud(cloud); // Crea el mapa / arbol

    for (const auto& punto : cloud->points) { // como hacer en python for i in lista. punto es i. points es la lista de dentro de cloud
        // Descarte rápido por geometría
        if (punto.x < 0.0 || punto.x > distancia_seguridad || std::abs(punto.y) > ancho_robot_medio) {
            continue;
        }

        // FILTRO DE SOLIDEZ DIRECTO
        std::vector<int> indices; // Los números de índice de los puntos que están cerca
        std::vector<float> distancias; A cuántos metros está cada uno de esos puntos.
        
        // Buscamos en un radio un poco mayor (10cm) 
        // Si hay más de 5 puntos no es ruido
        if (tree->radiusSearch(punto, 0.10, indices, distancias) > 5) {
            emergencia_freno = true;
            break; // ¡Encontrado! No hace falta mirar nada más.
        }
    }

    if (emergencia_freno) this->parar_robot();
}

void LocalizadorNode::ver_obstaculos_rviz2(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr obstaculos(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(cloud);

    for (const auto& punto : cloud->points) {
        if (punto.x < 0.0 || punto.x > distancia_seguridad) {
            continue;
        }

        // B. FILTRO DE RUIDO (Radius Outlier):
        // Buscamos cuántos vecinos tiene este punto en un radio de 10cm
        std::vector<int> indices_vecinos;
        std::vector<float> distancias_vecinos;
        
        // Si tiene más de 3 vecinos cerca, no es ruido, es un objeto sólido
        if (tree->radiusSearch(punto, 0.10, indices_vecinos, distancias_vecinos) > 5) {
            obstaculos->push_back(punto); // Guarda los puntos en la nueva nube de puntos
        }
    }

    // 3. Publicar la nube filtrada
    if (!obstaculos->empty()) {
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*obstaculos, output_msg);
        output_msg.header.stamp = this->get_clock()->now(); // Sello de tiempo actual
        output_msg.header.frame_id = "laser_frame";
        obstaculos_pub_->publish(output_msg);
    }
}

void LocalizadorNode::procesar_beacons(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
    // 1. Filtro de Intensidad (Solo lo que brilla)
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_brillante(new pcl::PointCloud<pcl::PointXYZI>);
    for (const auto& punto : cloud->points) {
        if (punto.intensity > 2000.0) {
            cloud_brillante->push_back(punto);
        }
    }

    if (cloud_brillante->empty()) return;

    // 2. Crear el buscador KdTree para los puntos brillantes
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(cloud_brillante);

    // 3. Configurar el Euclidean Clustering
    std::vector<pcl::PointIndices> cluster_indices; // Lista de los indices de los resultados. Se guarda indices para ligereza
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec; // El agrupador que contiene toda la lógica para decidir qué puntos van juntos y cuáles no.
    ec.setClusterTolerance(0.05); // Puntos a menos con una distancia menor de 5cm entre ellos forman el mismo objeto
    ec.setMinClusterSize(5);      // Al menos 5 puntos para ser una baliza
    ec.setMaxClusterSize(50);     // Si tiene muchos puntos, es una pared, no una baliza
    ec.setSearchMethod(tree);     // Le pasas el mapa para que sepa el orden y no lo haga a lo loco desordenado
    ec.setInputCloud(cloud_brillante); // Le pasas la nube con los datos
    ec.extract(cluster_indices); // El botón de EJECUTAR. El algoritmo empieza a saltar de punto en punto usando el mapa y llena la lista cluster_indices con los grupos que haya encontrado.

    // 4. Analizar cada grupo con Eigen
    for (const auto& indices_grupo : cluster_indices) {
        // Extraemos los puntos de este cluster a un objeto Eigen para calcular rápido

        // min_pt pones x,y en un valor absurdamente alto para que en la primera el primer valor pase a ser el nuevo minimo. Igual con el max pero al reves.
        // Como si estuvieras definiendo dos variables en una misma linea -> float num1, num2;
        Eigen::Vector2f min_pt(999, 999), max_pt(-999, -999); 

        for (auto idx : indices_grupo.indices) {
            const auto& p = cloud_brillante->points[idx];
            if (p.x < min_pt.x()) min_pt.x() = p.x;
            if (p.y < min_pt.y()) min_pt.y() = p.y;
            if (p.x > max_pt.x()) max_pt.x() = p.x;
            if (p.y > max_pt.y()) max_pt.y() = p.y;
        }

        // Calculamos el ancho del objeto usando la distancia euclidiana de Eigen
        float ancho = (max_pt - min_pt).norm();

        // 5. ¿Mide los 10 cm sagrados? (Damos un margen de error de 3cm)
        if (ancho > 0.07 && ancho < 0.13) {
            float centro_x = (min_pt.x() + max_pt.x()) / 2.0;
            float centro_y = (min_pt.y() + max_pt.y()) / 2.0;
            
            RCLCPP_INFO(this->get_logger(), "¡Baliza detectada en: X=%.2f, Y=%.2f!", centro_x, centro_y);
            // Aquí guardarías estos centros para la triangulación final
        }
    }
}

int main(int argc, char ** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathFollower>(); 

    try {
        node.spin();
    }
    catch (const std::exception & e) {
        RCLCPP_ERROR(node->get_logger(), "Error en el ejecutor: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}