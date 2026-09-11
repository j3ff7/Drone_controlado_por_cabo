import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'pacote_do_drone'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Instala os arquivos de Launch
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        
        # Procura qualquer .sdf dentro da pasta worlds e instala na pasta share do pacote
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        
        # Instala os parâmetros junto com a estrutura de diretórios do pacote
        (os.path.join('share', package_name, 'parameters'), glob('parameters/*.json')),

        # Instala arquivos na pasta models (incluindo o cabo.sdf)
        (os.path.join('share', package_name, 'models'), glob('models/*.*')),
        
        # Instala a pasta Gazebo (onde o Python salva o cabo agora)
        (os.path.join('share', package_name, 'models/Gazebo'), glob('models/Gazebo/*.*')),
        
        # Instala a pasta do drone e suas malhas (meshes)
        (os.path.join('share', package_name, 'models/carretel'), glob('models/carretel/*.*')),
        (os.path.join('share', package_name, 'models/meu_drone'), glob('models/meu_drone/*.*')),
        (os.path.join('share', package_name, 'models/meu_drone/meshes'), glob('models/meu_drone/meshes/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seu_nome',
    maintainer_email='seu_email@todo.todo',
    description='Pacote do drone para Iniciação Científica',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensores = pacote_do_drone.sensores:main',
            'mission_test1 = pacote_do_drone.mission_test1:main',
        ],
    },
)
