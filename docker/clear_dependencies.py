import re
import sys


def remover_find_package_exceto_catkin(file_path):
    """
    Remove todos os comandos find_package(pacote) de um arquivo CMakeLists.txt,
    exceto aqueles que correspondem a find_package(catkin).
    Args:
        caminho_arquivo (str): O caminho para o arquivo CMakeLists.txt.
    """
    with open(file_path, 'r') as f:
        content = f.read()

    # Expressão regular para encontrar e remover find_package(pacote)
    # que não seja find_package(catkin).
    pattern = re.compile(r'find_package\((?![ \t]*catkin[ \t)]).*\)')
    new_content = pattern.sub('', content)

    with open(file_path, 'w') as f:
        f.write(new_content)
    

def clean_catkin_dependencies(file_path):

    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # Regex para encontrar o comando find_package(catkin ...)
    pattern = re.compile(
        r'find_package\s*\(\s*catkin\s+(?:REQUIRED\s*)?'  # find_package( catkin REQUIRED (opcional)
        r'(?:COMPONENTS\s*)?'                             # COMPONENTS (opcional)
        r'([^)]*?)'                                       # Captura os componentes (non-greedy)
        r'\s*\)',                                         # Fecha o parêntese
        re.IGNORECASE | re.DOTALL                       
    )

    # Lista para armazenar as linhas do novo conteúdo
    new_content_lines = []
    last_end = 0

    for match in pattern.finditer(content):
        start, end = match.span()
        components_str = match.group(1)

        # Adiciona a parte do conteúdo antes do find_package atual
        new_content_lines.append(content[last_end:start])

        # Extrai os componentes, lidando com espaços e quebras de linha
        raw_components = re.split(r'\s+', components_str.strip())
        
        # Filtra os componentes
        filtered_components = [
            comp for comp in raw_components
            if comp.endswith('_msgs') or comp == 'message_generation'
        ]
        
        # Reconstrói a linha find_package com os componentes filtrados
        # Mantém a formatação original da linha find_package
        if filtered_components:
            prefix = content[start:match.start(1)] 
            line_start_index = content.rfind('\n', 0, match.start()) + 1
            indentation = content[line_start_index:start].split('find_package')[0]
            formatted_components = "\n".join([f"{indentation}  {comp}" for comp in filtered_components])
            new_find_package_line = f"{prefix}{formatted_components})\n"
            new_content_lines.append(new_find_package_line)
        else:
            new_content_lines.append("")
            
        last_end = end

    # Adiciona qualquer conteúdo restante após o último find_package
    new_content_lines.append(content[last_end:])
    new_content = "".join(new_content_lines)

    with open(file_path, 'w', encoding='utf-8') as f:
        f.write(new_content)

def filter_catkin_package(file_path):
    # Expressão regular para encontrar a diretiva CATKIN_DEPENDS e seus valores
    catkin_depends_pattern = re.compile(r'CATKIN_DEPENDS\s*(.*)', re.DOTALL)

    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()
        match = catkin_depends_pattern.search(content)

        if not match:
            print("Não foi encontrada a diretiva CATKIN_DEPENDS")
            return None
        
        catkin_depends_content = match.group(1).strip()
        # Divide os pacotes e filtra
        packages = catkin_depends_content.split()
        filtered_packages = [
            pkg for pkg in packages
            if pkg.endswith('_msgs') or pkg == 'message_runtime'
        ]
        
        # Reconstrói a string CATKIN_DEPENDS
        if filtered_packages:
            new_catkin_depends = f'CATKIN_DEPENDS {" ".join(filtered_packages)}'
        else:
            new_catkin_depends = '' # Se não houver pacotes válidos, remove a diretiva

        # Primeiro, pegamos o conteúdo entre 'catkin_package(' e o ')' final
        content_within_parentheses_match = re.search(
            r'catkin_package\((.*)\)', content, re.DOTALL
        )

        if not content_within_parentheses_match:
            print("Erro! Não achei o catkin_package")

        # Adiciona a nova CATKIN_DEPENDS, se houver
        if new_catkin_depends:
            # Garante que haverá apenas o CATKIN_DEPENDS e nada mais
            new_catkin_package_str = f'catkin_package(\n    {new_catkin_depends}\n)'
        else:
            # Se não houver pacotes, o resultado deve ser catkin_package( )
            new_catkin_package_str = 'catkin_package()'
        
        start_index = content_within_parentheses_match.start()
        end_index = content_within_parentheses_match.end()

        # Substitui a catkin_package original no conteúdo total do arquivo
        modified_content = content[:start_index] + new_catkin_package_str + content[end_index:]

        with open(file_path, 'w', encoding='utf-8') as f:
            f.write(modified_content)

# Exemplo de uso:
if __name__ == "__main__":
    clean_catkin_dependencies(sys.argv[1])
    remover_find_package_exceto_catkin(sys.argv[1])
    filter_catkin_package(sys.argv[1])