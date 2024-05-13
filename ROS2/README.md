# ROS2

Olá, seja bem vindo ao tutorial de ROS2 do Pequi Mecânico, nosso objetivo é introduzir essa aplicação tão importante no nosso meio e mostrar alguns dos seus principais conceitos.

## Introdução

O ROS2, ou Robot Operating System 2, é um framework open-source projetado para o desenvolvimento de software para robôs. Ele oferece uma estrutura flexível e modular que permite aos desenvolvedores criar e gerenciar sistemas robóticos complexos.

## Instalação

Para instalar o ROS2, siga o tutorial disponível na documentação, ele é realmente prático e simples de aplicar, aqui um exemplo para instalação do ROS2 Humble: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

## Principais conceitos

Para entender como o ROS é estruturado, o entendimento de seus conceitos é o passo mais importante, vamos tentar explicá-los de forma resumida.

### Nó

Dentro do ROS2, um nó é uma unidade fundamental no sistema, representando um processo independente que executa tarefas específicas. Os Nós são essenciais para a arquitetura do ROS2, permitindo a modularidade e a distribuição de funcionalidades em um sistema robótico.<br><br>
Cada nó é executado como um processo independente, proporcionando isolamento e facilitando o desenvolvimento modular. Isso permite que diferentes partes de um sistema robótico sejam desenvolvidas, testadas e modificadas independentemente.<br><br>
Abaixo temos um exemplo da criação de um nó simples em python:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):

    def __init__(self):
        super().__init__('fla_node')
        self.get_logger().info('Flamengo é o maior time do mundo!')
        

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
```

### Mensagem

Em ROS2, uma mensagem é uma estrutura de dados usada para trocar informações entre Nós através de tópicos. As mensagens definem o formato dos dados que estão sendo transmitidos, permitindo que Nós diferentes comuniquem informações específicas de maneira padronizada. Cada tópico em ROS2 é associado a um tipo de mensagem particular.<br><br>
As mensagens são definidas em arquivos específicos conhecidos como .msg files (arquivos de mensagem) e seguem uma sintaxe específica. O conteúdo desses arquivos descreve a estrutura dos dados que será transmitida pelo tópico. Exemplos de dados incluem valores numéricos, strings, matrizes e outros tipos de dados personalizados.<br><br>
A biblioteca padrão de mensagens em ROS2 é chamada "std_msgs", e ela inclui mensagens comuns, como "String", "Int32", "Float64", entre outras. Além disso, os desenvolvedores podem criar mensagens personalizadas para atender às necessidades específicas de seus projetos.

### Tópico

Um tópico é um canal de comunicação unidirecional no qual os nós podem publicar mensagens ou se inscrever para receber mensagens. Os nós que desejam compartilhar informações sobre um determinado tópico publicam mensagens nele, enquanto os nós interessados em receber essas informações se inscrevem no mesmo tópico.<br><br>
A comunicação entre nós por meio de tópicos é assíncrona. Isso significa que os nós não precisam esperar por uma resposta imediata ao enviar uma mensagem. Além disso, cada tópico está associado a um tipo específico de mensagem.<br><br>
Para criar tópicos, é necessário o entendimento dos conceitos de publisher e subscriber.

![Alt Text](https://docs.ros.org/en/humble/_images/Topic-SinglePublisherandSingleSubscriber.gif)

#### Publisher

Um Publisher cria e publica mensagens em um tópico específico. Ele está associado a um tipo de mensagem e fornece uma interface para enviar dados a outros nós interessados no mesmo tópico. Abaixo um exemplo de como criar um publicador.

```python
class MinimalPublisher(Node):
 
 def __init__(self):
    # calling the Node class constructor and naming the Node
    super().__init__('minimal_publisher')
    # declares node publishes message type String, over the topic named topic, and 
    # queue size is 10
    self.publisher_ = self.create_publisher(String, 'topic', 10)
```

#### Subscriber

Um Subscriber cria uma assinatura (subscription) para um tópico específico e especifica uma função de retorno de chamada (callback) que será executada sempre que uma nova mensagem for recebida. Ele consome mensagens publicadas no tópico associado. Abaixo um exemplo de subscrição em um tópico.

```python
class MinimalSubscriber(Node):
 
 def __init__(self):
    super().__init__("minimal_subscriber")
    self.subscription = self.create_subscription(String, 'topic', self.listener_callback, 10)

 def listener_callback(): 
  ...
```

### Serviço

Em ROS2, um serviço é um mecanismo de comunicação que permite a troca de mensagens síncronas entre nós. Ao contrário dos tópicos, que são utilizados para comunicação assíncrona, os serviços são utilizados para comunicação síncrona, onde um nó solicita um serviço e aguarda uma resposta.<br><br>
Um nó que faz uma solicitação de serviço é chamado de cliente de serviço (Service Client). O cliente cria uma solicitação, a envia para o nó que oferece o serviço e aguarda a resposta.<br><br>
Um nó que oferece um serviço é chamado de servidor de serviço (Service Server). O servidor recebe solicitações do cliente, processa essas solicitações e envia uma resposta de volta ao cliente.<br><br>

![Alt Text](https://docs.ros.org/en/humble/_images/Service-SingleServiceClient.gif)

Exemplo de criação de um serviço:

```python
from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node


class FlamengoService(Node):

    def __init__(self):
        super().__init__('flamengo_service')
        self.srv = self.create_service(AddTwoInts,
                                       'add_two_ints', 
                                        self.flamengo_callback)

    def flamengo_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response


def main():
    rclpy.init()

    flamengo_service = FlamengoService()

    rclpy.spin(flamengo_service)

    rclpy.shutdown()
```

### Ação

Em ROS 2, uma ação é um mecanismo de comunicação mais avançado do que tópicos e serviços, projetado para facilitar a execução de tarefas assíncronas que podem levar um tempo considerável para serem concluídas. As ações são usadas quando uma tarefa específica exige uma interação mais complexa entre nós, especialmente quando há uma expectativa de feedback contínuo durante a execução da tarefa.<br><br>
As ações têm três componentes principais: Goal (objetivo), Result (resultado) e Feedback (feedback). O nó cliente envia um objetivo para o nó servidor (Goal), e o nó servidor fornece feedback contínuo durante a execução da tarefa. Após a conclusão da tarefa, o servidor envia o resultado de volta ao cliente.<br><br>
As ações suportam timeouts, o que significa que o cliente pode especificar quanto tempo está disposto a esperar pela conclusão da tarefa. Além disso, os clientes podem solicitar o cancelamento de uma ação em andamento.<br><br>

![Alt Text](https://docs.ros.org/en/humble/_images/Action-SingleActionClient.gif)

Exemplo para criação de uma ação:

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci


class FlamengoActionServer(Node):

    def __init__(self):
        super().__init__('flamengo_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci, # tipo de action
            'flamengo', # nome da ação
            self.execute_callback) # método que retorno result

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        result = Fibonacci.Result()
        return result


def main(args=None):
    rclpy.init(args=args)

    flamengo_action_server = FlamengoActionServer()

    rclpy.spin(flamengo_action_server)
```

## Conclusão

Agora que você conhece os principais conceito do ROS, a maior dica que podemos dar é partir para a prática, sinta-se a vontade para adicionar suas conclusões neste repositório e contribuir o seu desevolvimento.

[ROS2 Tutorials](https://www.youtube.com/playlist?list=PLLSegLrePWgJudpPUof4-nVFHGkB62Izy)
