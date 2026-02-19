# Cria uma pasta temporária para baixar o código fonte
function clone_if_not_available() {
    if [ ! -d "$1" ]; then
        if ! git clone --recurse --depth=1 -b $2 git@bitbucket.org:otmisdev/$1.git
        then
            echo "Não foi possível clonar o repositório $1. Você tem acesso a esse repositório?"
            return 
        else
            echo "Repositório $1 clonado com sucesso"
        fi
    else
        echo "Repositório $1 já clonado"
    fi
}

function create_and_go() {
    if [ -d "./$1" ]; then
        cd $1
    else
        mkdir $1 && cd $1
    fi
    echo "No diretório $1"
}

function clear_source() {
    if [ -d "./$1" ]; then
        rm -rf $1
    fi
}

function remove_non_msg_pkgs() {
    find . -type d -name "*msgs" -exec sh -c '
    for dir do
        BASENAME=$(basename "$dir")
        DEST_DIR=.
        echo "Movendo a pasta $BASENAME de $dir para $DEST_DIR"
        mv "$dir" "$DEST_DIR/"
    done
' _ {} +
    rm -rf !(*msgs)
}

function clean_CMakeLists() {
    for var in "$@"; do
        find $var -type f -name "CMakeLists.txt" -print0 | while IFS= read -r -d $'\0' file; do
            echo "Processando: $file"

            # Define a versão do C++ para 17
            update_CMakeLists_cpp_version $file
            
            # Remove comandos de build e instalação
            remove_CMakeLists_builds $file
            
            # Remove dependências não usadas
            remove_catkin_unused_dependencies $file

            if [ $? -eq 0 ]; then
                echo "  Sucesso! '$file' editado."
            else
                echo "  Erro: Não foi possível editar '$file'."
            fi
        done
    done
}


function update_CMakeLists_cpp_version() {
    echo "  Configurando versão do C++ para 17..."
    # Substitui -std=c++11 por -std=c++17
    sed -i 's/-std=c++11/-std=c++17/g' "$1"
    # Substitui -std=c++14 por -std=c++17
    sed -i 's/-std=c++14/-std=c++17/g' "$1"
    # Substitui CMAKE_CXX_STANDARD 11 por CMAKE_CXX_STANDARD 17
    sed -i 's/CMAKE_CXX_STANDARD 11/CMAKE_CXX_STANDARD 17/g' "$1"
    # Substitui CMAKE_CXX_STANDARD 14 por CMAKE_CXX_STANDARD 17
    sed -i 's/CMAKE_CXX_STANDARD 14/CMAKE_CXX_STANDARD 17/g' "$1"
    echo "  Configurado para C++ 17!"
}

function remove_CMakeLists_builds() {

    # Lista de comandos a serem removidos
    COMMANDS=(
        "add_dependencies"
        "add_executable"
        "catkin_install_python"
        "file"
        "generate_dynamic_reconfigure_options"
        "include_directories"
        "install"
        "target_link_libraries"
        "roslint_cpp"
        "if"
        "else"
        "endif"
        "pkg_check_modules"
    )
    
    echo "  Removendo instalações e builds..."
    # Loop para remover cada comando
    for CMD in "${COMMANDS[@]}"; do
        # Remove se o parênteses está fechado na msm linha
        sed -i "/^ *${CMD}([^)]*)/d" "$1" 
        # Remove se o parênteses está fechado em uma linha seguinte
        sed -i "/^ *${CMD}(/,/)/{//!d; /)/d; /^ *${CMD}(/d}" "$1"
    done
    
    echo "  Instalações e builds removidas!"
}

function remove_catkin_unused_dependencies() {

    FILE="$1"
    echo "  Limpando dependências..."
    python3 ../docker/clear_dependencies.py $FILE
    echo "  Limpeza de dependências concluída!"
}