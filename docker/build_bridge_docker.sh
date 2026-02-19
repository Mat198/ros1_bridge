#!/bin/bash
source ./docker/functions_build.sh

if [[ "$*" != *"--skip-deps"* ]]; then
    echo "Clonando dependências..."

    # Adiciona repositórios com msgs do ROS 1
    create_and_go ros1_msgs
    
    # Clona repositórios externos 
    git clone -b master https://github.com/ANYbotics/grid_map.git
    cd grid_map
    git checkout 381b30ecefd261cbfc8f1d296dac6b587a062c28
    cd ..

    git clone -b master https://github.com/ANYbotics/kindr_ros.git
    cd kindr_ros
    git checkout 2bf7ba630b381f4f294cb0c88684479dc6a29618
    cd ..

    git clone --depth=1 -b master https://github.com/ros-geographic-info/geographic_info.git

    git clone --depth=1 -b master https://github.com/ros-geographic-info/unique_identifier.git

    git clone --depth=1 -b noetic-devel https://github.com/ros/ros_comm_msgs.git
    cd ros_comm_msgs
    rm -r rosgraph_msgs # Já foi incluída. Duplicar vai gerar problemas. Queremos apenas std_srvs
    cd ..

    # Remove os pacotes que não são de mensagens para reduzir conflitos
    remove_non_msg_pkgs

    # Clona repositórios externos sem pacote de mensagem exclusivo
    git clone --depth=1 -b noetic-devel https://github.com/cra-ros-pkg/robot_localization.git

    # Realiza a limpeza dos arquivos CMakeList
    #   Atualiza a versão do C++ para evitar erros de compilação
    #   Remove dependências que não são de mensagens
    #   Remove build e instalação de executáveis e libraries
    #   Remove diretivas do catkin_package() com exceção do CATKIN_DEPENDS
    clean_CMakeLists \
        ouster_ros robot_localization
    cd ..

    # Adiciona repositórios com msgs do ROS 2 
    create_and_go ros2_msgs

    # Dependências externas
    git clone -b master https://github.com/ANYbotics/kindr_ros.git
    git clone -b ros2 https://github.com/ouster-lidar/ouster-ros.git

    remove_non_msg_pkgs
    cd ..
fi

# Builda o docker
if ! docker build -t ros1_bridge:latest . -f docker/Dockerfile.bridge
then
    echo "Falhou ao buildar o docker"
fi

if [[ "$*" != *"--keep-pkgs"* ]]; then
    echo "Removendo arquivos fonte..."
    clear_source temp
    clear_source ros1_msgs
    clear_source ros2_msgs
else
    echo "Pacotes fonte foram mantidos"
fi
