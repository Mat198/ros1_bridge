#!/bin/bash

# Função para exibir instruções de uso
show_help() {
    echo "Uso: $0 <caminho_para_o_arquivo.bag>"
    echo ""
    echo "Descrição:"
    echo "  Converte um arquivo bag do ROS 1 para ROS 2 utilizando um container Docker."
    echo ""
    echo "Exemplo:"
    echo "  $0 ./meu_diretorio/dados_sensor.bag"
}

# 1. Verifica se algum argumento foi fornecido
if [ -z "$1" ]; then
    show_help
    echo "Erro: Nenhum arquivo fornecido."
    return 1
fi

# 2. Verifica se o arquivo existe fisicamente
if [ ! -f "$1" ]; then
    echo "Erro: O arquivo '$1' não foi encontrado ou não é um arquivo comum."
    return 1
fi

# 3. Verifica se a extensão é .bag (case-insensitive)
if [[ ! "$1" =~ \.[Bb][Aa][Gg]$ ]]; then
    echo "Erro: O arquivo fornecido não possui a extensão .bag"
    return 1
fi

# Nome base do bag
ROS1_BAG_FILE=$(basename "$1")
ROS1_BAG_DIR=$(dirname "$1")

# Remove a extensão para usar como nome do ROS 2
ROS1_BAG_NAME="${ROS1_BAG_FILE%.bag}"

echo "--------------------------------------------------------"
echo "Convertendo: $ROS1_BAG_FILE"
echo "Diretório:   $(realpath "$ROS1_BAG_DIR")"
echo "--------------------------------------------------------"

docker run -it --rm --network=host  \
    -v "$(realpath $ROS1_BAG_DIR):/bags" \
    --name=bag_converter ros1_bridge:latest \
    /convert_bag.sh /bags/$ROS1_BAG_FILE /bags/$ROS1_BAG_NAME

# Verifica se o comando Docker falhou
if [ $? -eq 0 ]; then
    echo "Sucesso: Conversão concluída para o diretório /bags/$ROS1_BAG_NAME"
else
    echo "Erro: Falha na conversão via Docker."
    return 1
fi