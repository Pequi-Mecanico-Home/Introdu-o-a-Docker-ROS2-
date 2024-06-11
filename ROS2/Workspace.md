# Documentação ROS2

Este repositório foi criado pelo nosso coleguinha João Vitor Costa. Tem foco na criação e uso de um Workspace.

# Sumário

- Introdução
- Criando um Work Space e um Repositório
- The ros Transform System
- Descrição do nosso robô com um URDF
- Simulação com o gazebo

# Introdução

Nossos companheiros de equipe já fizeram uma documentação de introdução do ros2, portanto caso você precise dessa pequena introdução acesse este link:

[](https://github.com/Pequi-Mecanico-Home/Introducao-Docker-e-ROS2/tree/dev/ROS2)

O que teremos de novo aqui será o conceito de launch. Independentemente da versão específica, um "launch" no contexto do ROS 2 se refere a um arquivo de lançamento (launch file). O arquivo de lançamento é usado para iniciar vários nós do ROS 2 e configurar parâmetros, namespaces, remapeamentos e outras configurações do ROS em um único arquivo.

Os arquivos de lançamento são geralmente escritos em XML e podem incluir a inicialização de nós, configurações de namespaces, remapeamentos de tópicos, argumentos e outros detalhes de configuração. Eles oferecem uma maneira conveniente de iniciar e configurar várias partes de um sistema ROS 2 com um único comando.

Esssa é a maneira mais comum de rodarmos um launch

```bash
ros2 launch meu_pacote meu_arquivo_de_lancamento.py
```

# Criando um Work Space e um Repositório

Primeiramente criamos um diretório para trabalharmos em cima do ros no qual devemos denomina-lo como:

```bash
mkdir -p example_of_ws/src
```

Depois disso transformamos ele em um workspace (ws):

```bash
colcon build --symlink-install
```

O colcon, por padrão irá criar os seguintes diretórios como pares do diretório src:
O diretório build sera onde os arquivos intermediários seram arquivados. Para cada pacote uma subpasta sera criada onde o CMake será invocado.
O diretório install é onde cada pacote será instalado. POr padrão cada pacote será instalado dentro de subdiretórios separados. 
O diretório Log contém várias informações em log sobre cada chamado do colcon. 


Dentro do nosso ws/src criamos um pacote vazio:

```bash
ros2 pkg create --build-type ament_cmake my_package_example
# --build-type é uma flag que depois irei passar o tipo de pacote que será buildado neste caso ament_cmake
# depois digo o nome do pacote neste caso my_package_example
```

Com isso ele vai incluir os seguintes diretórios no meu ws log, install, build e src. Além disso, dentro do diretório src vai vim de padrão dois arquivos que é o package.xml e o CMakeLists.txt.

 O package.xml vai fornecer informações cruciais para o sistema entender como ele vai compilar, instalar e executar o pacote. 

No CMakeLists.txt vamos ter:

- Definições do pacote(nome, versão, licença);
- Como deve ser feita a compilação;
- Dependências;
- Configuração de dependências;
- Instalações nescessárias.

A partir daqui recomendamos abrir o ws no VScode para trabalharmos de forma mais confortável.

Devemos fazer a seguinte alteração no CMakeLists.txt:

```bash
cmake_minimum_required(VERSION 3.8)
project(jv_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}) # ALTERAÇÃO

ament_package()
```

No arquivo package.xml devemos fazer as seguintes alterações também:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>jv_package</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="costajoaovitorsouza2004@gamil.com">jv_costa</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <exec_depend>demo_nodes_cpp</exec_depend>
	<exec_depend>demo_nodes_py</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

Dentro de src crie um diretório chamado launch com doi arquivos .py chamados [talker.launch.py](http://talker.launch.py) e listener.launch.py

Em [talker.launch.py](http://talker.launch.py) e [listener.launch.py](http://listener.launch.py) teremos um nó com um pacote e um executável:

```python
//talker.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package = 'demo_node_cpp',
            executable = 'talker'
        )
    ])
```

```python
//talker.listener.py

from launch import LaunchDescription
from launch_ros.actions import Node 

def generate_launch_description():
    return LaunchDescription([

        Node(

            package = 'demo_nodes_py',
            executable = 'listener'
        )
    ])
```

Após estes passos vamos criar um repositório no github para termos melhor controle do projeto.

Siga os seguintes passos:

```bash
# Você deve estar no repositório que quer transformar em repositório

	git init
  git add . # o ponto adiciona todo o diretório
  git commit -m "first commit"
  git branch -M main
  git remote add origin git@github.com:jv-costa/teste.git #jv-costa/teste é o diretório no seu caso será diferente
  git push -u origin main
```

## The ROS Transform System (TF)

A transformação de sistemas altera um frame para outro frame.

O que isso quer dizer? Pensando de for matemática temos a possibilidade de alterar nosso sistema de coordenada onde cada robô possui um diferente além do ponto referencial do ambiente. Conhecendo o sitemas de coordenadas de cada um e a posição de  um determinado objeto em relação à apenas um desses componentes(robôs ou ponto de referência do ambiente) podemos fazer a transformação de um sistemas de coordenas para outro encontrando a posição do objeto em relação ao outro robô no qual não tinhamos ideia da posição.

obs: o robô é apenas um exemplo veja nessa imagem por exemplo uma outra aplicação.

![Untitled](Documentac%CC%A7a%CC%83o%20ROS2%20dea2ae9aeea84243ab6a03c952bb4b89/Untitled.png)

Temos de concordar que é um tanto quanto confuso, mas para nossa sorte o ROS faz todo o trabalho pesado e já nos entrega algo relativamente pronto.

### Broadcasting Static TF’s:

Como de se esperar essa ferramenta calcula TF’s de sistemas estáticos. Essa é a ideia do comando que devemos executar 

ros2 run tf2_ros static_transform_publisher x y z yaw pitch roll parent_frame child_frame

Vamos usar o seguinte exemplo:

![Untitled](Documentac%CC%A7a%CC%83o%20ROS2%20dea2ae9aeea84243ab6a03c952bb4b89/Untitled%201.png)

![Untitled](Documentac%CC%A7a%CC%83o%20ROS2%20dea2ae9aeea84243ab6a03c952bb4b89/Untitled%202.png)

Em um terminal digite o comando:

```bash
ros2 run tf2-ros static_transform_publisher 2 1 0 0.785 0 0 world robot_1
```

Em outro digite:

```bash
ros2 tf2_ros static_transform_publisher1 0 0 0 0 0 robot_1 robot_2
```

Agora abra o rviz2:

```bash
ros2 run rviz2 rviz2
```

No rviz siga esses passos:

- Clique em add no canto inferior direito e selecione TF
- Clique em Fixed Frame no canto superior esquerdo e selecione world

Agora você pode ver melhor o que está acontecendo.

### Broadcasting Dynamic TF’s:

Agora teremos um caso se um sistema dinâmico.

```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$( xacro ~/example_robot.urdf.xacro )"
```

Nesse novo comando adicionamos um novo arquivo .xacro nele temos a descrição do nosso objeto que queremos trabalhar. E para rodar esse comando use este arquivo.

[example_robot.urdf.xacro](Documentac%CC%A7a%CC%83o%20ROS2%20dea2ae9aeea84243ab6a03c952bb4b89/example_robot.urdf.xacro)

```bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

Este comando vai abrir uma interface gráfica que vai te permitir alterar características do robô exemplo 

Abra o rviz2 faça as alterações do exemplo passado e verifique as transformações.

Lembre que você pode alterar as transformações pela interface que abrimos anteriormente.

## Debugging with view_frames

Algumas vezes podemos ter problemas com essas transformações. Para solucionarmos de um maneira mais fácil podemos pedir uma série de dados nas quais podemos verificar se exste alguma anomalia. Rode o seguinte comando.

```bash
ros2 run tf2_tools view_frames.py
```

Com isso teremos um novo arquivo na sua home, nele você terá diversos dados de como foi feita as transformações para caso tenha que debugar.

## Descrição do nosso robô com um URDF

Primeiro de tudo o URDF significa **Unified Robot Description Format.** É um formato de arquivo xml usado para descrever a estrutura cinemática de um robô.

Este tipo de arquivo nos permite especificar informações como as dimensões dos links do robô, bem como outros detalhes importantes da cinemática, como limites de movimento e inércias.

Essa descrição é fundamental para o planejamento de movimento, simulação e controle de robôs em ambientes virtuais.

## Links e Joints:

Um “link” refer-se a uma parte ou elo físico do robô, Cada link representa uma parte específica do robô, como um braço, uma petrna, ou qualquer outra parte do corpo. Esses links são conectados por Joints que repesentam as articulações entre eles.

Em relação aos Joints podemos ter alguns tipos de Joints:

- **Fixed Joint**
    
    Permite que dois links sejam rigidamente conectados, sem nenhum movimento relativo entre eles.
    
- **Revolute Joint**
    
    Prmiote um movimento rotacional ao redor de um eixo fixo
    
- **Prismatic Joint**
    
    Permite um movmento translacional ao longo de um eixo fixo
    
- **Continuos Point**
    
    Similar à junta de revolução, mas sem limites, representando movimento rotacional contínuo
    
- **Flating Joint**
    
    Permite seis graus de liberdade (3 translacionais e 3 rotacionais), proprcionando um movimento mais complexo
    

## Sintax de um arquivo XML:

```xml
<?xml version="1.0"?>

<robot name="nome_do_robo">

  <!-- Definição dos links -->
  <link name="link1">
    <!-- Propriedades visuais e de colisão, inércia, etc. -->
  </link>

  <link name="link2">
    <!-- Propriedades visuais e de colisão, inércia, etc. -->
  </link>

  <!-- Definição das juntas -->
  <joint name="junta1" type="revolute">
    <!-- Parâmetros específicos da junta (eixo, limites, etc.) -->
    <parent link="link1"/>
    <child link="link2"/>
  </joint>

  <!-- Mais links e juntas podem ser adicionados conforme necessário -->

</robot>
```

- **<?xml version=”1.0”?>**
    
    Declaramos a versão do xml
    
- **<robot name=”nome_do_robo”>
</robot>**
    
    Dentro de todo esse elemento teremos toda a descrição do robô, e o atributo name fornece um nome único para ele.
    
- **<link name=”link1”>
</link>**
    
    Aqui teremos as informações sobre a geometria, inércia, propriedades visuais e de colião daqule determinado link 
    
    Acredito que nessas duas imagens você poderá ter uma ideia melhor:
    
    ![Untitled](Documentac%CC%A7a%CC%83o%20ROS2%20dea2ae9aeea84243ab6a03c952bb4b89/Untitled%203.png)
    
    ![Untitled](Documentac%CC%A7a%CC%83o%20ROS2%20dea2ae9aeea84243ab6a03c952bb4b89/Untitled%204.png)
    
- **<joint name=”joint1” type=”revolute”>
</joint>**
    
    Aqui temos a definição da relação do joint, ou seja, define qual link é o **pai** e qual é o **filho**, temos a origem, a subtag **axis** que nos define em qual dos eixos a junta poderá fazer rotação e temos a subtag **limit** onde definimos o limite máximo de alguns parâmetros como velocidade, a posição angular e linear máxima e mínima que pode ter entre outros.
    
    Acredito que nessa imagen você poderá ter uma ideia melhor:
    
    ![Untitled](Documentac%CC%A7a%CC%83o%20ROS2%20dea2ae9aeea84243ab6a03c952bb4b89/Untitled%205.png)
    

Podemos ter outras tags dentro do xml, mas as mais fundamentais são essas.

É legal seguirmos um certo padrão para determinar os nomes dos links e dos joints, como por exempo:

**arm_link**, **arm_joint**

Assim especificamos com o que estamos trabalhando.

## Usando xacro para facilitar as coisas:

O Xacro é uma linguagem de extensão do XML desenvolvida para simplificar a descrição de modelos URDF em ROS. A linguagem foi projetada para facilitar a criação e manutenção de descrições de robôs em XML.

- **Reutilização de Código**
    
    O Xacro permite a utilização de macros, que são pedaços reutilizáveis de código, facilitando a criação de modelos de robôs mais complexos e a manutenção de código mais limpo.
    
- **Permite Parametrização**
    
    Uma característica poderosa do Xacro é a capacidade de parametrizar modelosa. Isso significa que você pode definir parâmetros em um arquivo Xacro e usar esses parâmetros para ajustar dinamicamente as propriedades do seu modelo de robô.
    
- **Integração com ROS**
    
    O Xacro é frequentemente usado em conjunto com o ROS, especialmente para descrever modelos de robôs que serão utilizados em simulações ou em experimentos com hardware compatível.
    

Portanto, o Xacro é uma estensão do XML. Os arquivos Xacro são basicamente arquivos XML que incorporam funcionalidades adicionais.

## Simulação com o Gazebo

O Gazebo é um software de simulação que se entegra muito bem com o ROS pelo fato de serem criados pela mesma empresa.

Para baixar o gazebo use o comando:

```bash
sudo apt install ross-humble-gazebo-ros-pkgs
```

Inicie um mundo no gazebo:

```bash
gazebo /usr/share/gazebo-11/worlds/seesaw.worldf
```

Na aba world você vai ver os links que temos nesse mundo e também conseguiremos interagir com eles neste mundo. Podemos aplicar forças, mudar eles de lugar entre outras coisa.

O gazebo utiliza um tipo de arquivo diferente do URDF que é o STF, mas não precisamos nos preocupar com isso já que o gazebo faz essa converção para gente. O arquivo STF ao invés de conter o necessário parar a simulação do robô ele tambpem tem o que precisamos para a simulação do robô e do mundo em que ele está contido.

## Example - Spawning a robot in Gazebo

Entre no seu work space e não se esqueça de buildar ele e dar source.

Logo em seguida adicione um novo pacote dentro de src para podermos trabalhar:

```bash
jv_costa@JV-Costa:~$ cd costa_ws/src/
jv_costa@JV-Costa:~/costa_ws/src$ git clone https://github.com/joshnewans/urdf_example.git
```

Neste pacote teremos arquivos xacro alguns launchs e coisas do rviz recomendo dar uma olhada.

Agora precisamos do nosso robot_state_publisher neste caso precisamos do launch.

```bash
ros2 launch urdf_example rsp.launch.py
```

Abra o rviz e passe o caminho para a simulação.

```bash
rviz2 -d src/udf_example/view_robot.rviz
```

Abra também o tópico joint_state_publisher.

```bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

Agora que verificamos que tudo isso funcionou vamos para a simulação no gazebo.

Inicie o gazebo com o seguinte launch.

```bash
ros2 launch gazebo_ros gazebo.launch.py
```

gazebo_ros é um pacote que permite a integração entre ros e gazebo.

Agora inicialize o launch do robot state publisher:

```bash
ros2 launch urdf_example rsp.launch.py
```

Adicione o robô ao mundo:

```bash
ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity my_bot
```

O nó spawn_entity é o script responsável por spawnar o robô no mundo.

Feito tudo isso agora abra o rviz e brinque com o robô se tudo foi feito de maneira correta o que for feito no rviz deve ser “sentido” pelo gazebo:

```bash
rviz2 -d src/urdf_example/view_robot.rviz
```

Verifique se uma mensagem consegue ser publicada no tópico e ser lida da maneira correta:

```bash
ros2 topic pub -1 /set_joint_trajectory trajectory_msgs/msg/JointTrajectory  '{header: {frame_id: world}, joint_names: [slider_joint, arm_joint], points: [  {positions: {0.8,0.6}} ]}'
```

# END…