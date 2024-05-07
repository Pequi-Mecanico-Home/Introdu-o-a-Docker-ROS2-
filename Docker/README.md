# Introdução aos básicos

Antes de tudo, para entender alguns comando que não lhe são familiares no linux, acesse [aqui](https://ubuntu.com/tutorials/command-line-for-beginners#1-overview).

Vamos agora aprender um pouco de rede.


## Alguns básicos de rede
### Interface configuration
Listar todas as interfaces de rede ativas no seu sistema
```
ifconfig

```
Você pode analisar o ip da máquina se atentando ao inet da rede.

Você pode também utilizar o comando ip:
```
ip address show
```
### Security protocol
Transferir arquivos ou diretórios de um host para outro em uma rede
```
scp origem destino

```
Para a máquina que não é host, é necessário especificar seu ip pelo formato **usuario@ip**. Exemplo:

```
scp arquivo.txt usuario@servidor:/caminho/para/o/destino

```
### Secure Shell

 É um protocolo de rede seguro que permite a comunicação segura e remota com outros dispositivos em uma rede, ou seja, acessar outra máquina.

```
ssh usuario@endereco_do_servidor

```
Por padrão, a porta usada é a 22, mas você pode alterar isso com a flag **-p**:

```
ssh usuario@endereco_do_servidor -p {porta}

```




## Conceitos de Docker

Antes de tudo, é necessário que você entenda bastante sobre conceitos mais teóricos:
+ O que é docker e para que serve
+ O que é um container
+ O que é uma imagem
+ O que é um volume
  
Esses e mais conceitos podem ser respondidos na primeira mentade desse [vídeo](https://www.youtube.com/watch?v=pg19Z8LL06w&t=2347s), sugiro que dê uma olhada para tentar se familiarizar com esses termos, assim como aprender como instalar docker em sua máquina.

## Comandos básicos

### Listar imagens 
Mostra todas as imagens instaladas em seu sistema.
```
docker images
```

### Remover imagem

```
docker rmi {nome_da_imagem}
```

### Baixar uma imagem
Baixar uma imagem do Docker Hub
```
docker pull {nome_da_imagem}:{tag}
```
### Buildar uma imagem

A partir de um [Dockerfile](https://nickjanetakis.com/blog/differences-between-a-dockerfile-docker-image-and-docker-container)
```
docker build {nome_DockerFile}
```
Por padrão, é buildado o arquivo com nome DockerFile caso selecione o próprio diretório como {nome_DockerFile}

Caso queira espeficar a tag da imagem (por padrão latest), basta utilizar  a flag -t:

```
docker build {nome_DockerFile} -t {nome_da_imagem_a_ser_buildada}:{tag}
```

### Executar um container
Para iniciar um novo contêiner a partir de uma imagem.
```
docker run {nome_da_imagem}
```


### Ver containers 
Listar containers em execução:
```
docker ps
```

Caso queira ver inclusive aqueles parados, execute:
```
docker ps -a
```
### Executar comandos em um container


```
docker exec -it {id_do_container} bash
```
Neste exemplo, **bash** é o shell que será executado no contêiner. **-it** é a flag usada para indicar que você deseja interagir com o terminal interativo do contêiner

### Iniciar um container parado

```
docker start {id_do_container}
```

### Parar um container


```
docker stop {id_do_container}
```
### Remover um container


```
docker rm {id_do_container}
```
### Attach
Conectar o terminal atual do seu host ao terminal de um contêiner Docker em execução (isso permite ver os outputs, erros etc do container em execução)
```
docker attach {id_container}

```
### Comandos úteis

#### CTRL + L - Limpar o terminal
#### CTRL + C - Interromper um processo
#### CTRL + D - Sair de um container
#### CTRL + P - Dettach de um container sem interrompê-lo
#### CTRL + Q - Sair de um container sem interrompê-lo


## As famosas flags em run

### -t/ --tty:
Criação de um pseudoterminal, sobre o container mas ao sair do terminal não haverá interação (terminal fica travado até pararmos o container)

### -i / --interactive
Permite inserir inputs no container

### -it:
Interactive tty , permite você acesse dentro do container através do terminal (usar comandos, ver outputs etc)

### --detach / -d:
Rodará o container em segundo plano e imprime na tela o hash do container

### --volume ou -v:
Permite que você crie uma conexão entre o arquivo do host para um arquivo do container. Assim, todas as alterações que você fizer dentro dessa pasta em seu container serão salvas no próprio host, mesmo fechando o container.
```
docker run pasta/no/host:pasta/no/container imagem
```

### --name:
Permite você dar um nome específico ao container
```
docker run --name {nome_do_container} imagem
```

### --restart:
Reinicia o container sempre que ele para (--restart always) ou falha(--restart on-failure:num_failures) 

### --rm:
Apaga o container quando for encerrado (útil para economizar espaço).

### --publish / -p:
Publica a porta de um container, permite que você controle explicitamente a correspondência de portas entre o host e o container.
```
docker run -p porta_host:porta_container imagem
```
### - P (uppercase):
Publica as portas de um container, mas deixa o Docker lidar automaticamente com a atribuição de portas no host para os portas expostos pelo container.
Para mais detalhes entre a diferenca de -p e -P, [acesse aqui](https://www.mend.io/free-developer-tools/blog/docker-expose-port/).

```
docker -P run imagem
```

### --ipc
Interprocess comunication, permite controlar o compartilhamento do namespace IPC entre containers. Ou seja,permite que os processos dentro do container se comuniquem diretamente com os processos do host. 

```
docker run --ipc=host imagem
```
### -net / --network
Permite especificar a configuração de rede para um container. É possível definir em qual rede o container será conectado.
```
docker run --network=host imagem
```
### --privileged
Concede ao container acesso total aos dispositivos, desativa muitos dos recursos de segurança que o Docker normalmente impõe. Útil paara certas tarefas que requerem acesso direto a dispositivos do sistema, ou realizar tarefas de baixo nível, mas deve ser usado com cautela, pois aumenta o potencial de riscos de segurança. Para mais detalhes do tema sobre acesso a dispositivos, [acesse aqui](https://www.youtube.com/watch?v=uf4zOigzTFo&t=400s&ab_channel=ArticulatedRobotics)


### --memory
Permite limitar a quantidade de memória RAM que um contêiner pode consumir
```
docker run --memory=2g imagem
```

### --unlimit
Permite que você estabeleça restrições em vários recursos do sistema, como memória, descritores de arquivo e CPU, para processos em execução dentro do container.

### --gpus
Permite especificar quantos recursos de GPU um contêiner pode acessar.

```
docker run --gpus <valor> imagem
```
ou até mesmo 
```
docker run --gpus all imagem
```

### runtime
Definir o tempo de execução (runtime) para um container específico.


```
docker run --runtime=<nome> imagem

```



## GUI
Para obter acesso a uma interface gráfica dentro de seu container, faça com que a partir de uma imagem Docker definindo a variável de ambiente DISPLAY com ```-e DISPLAY=$DISPLAY``` e o volume com ```-v /tmp/.X11-unix:/tmp/.X11-unix```. 

```
docker run -v /tmp/.X11-unix:/tmp/.X11-unix:rw --gpus all -e DISPLAY=$DISPLAY -it {nome_da_imagem}

```

Também certifique-se de que os clientes X possam ser conectados de qualquer host, incluindo nosso contêiner Docker, executando ```xhost +```.

[O que é x11?](https://www.baeldung.com/linux/x11).


---

[DOCUMENTAÇÃO OFICIAL DOCKER](https://docs.docker.com/engine/reference/run/)
