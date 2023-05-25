
## BUG2 Stage Controller

Este repositório traz uma implementação do algoritmo BUG2 para a navegação de um robô móvel no ROS e simulador Stage.

Contido nele, está o programa do BUG2, o programa para rodar ele, o arquivo do mundo para o simulador e o arquivo pra executar tudo.

Primeiramente, é necessário que se tenha o ROS Noetic instalado, juntamento do Stage.
Pode-se intalar o Stage pelo comando:

    sudo apt install ros-noetic-stage-ros

Essa implementação faz uso de uma interface *gym* para o recebimento e envio das mensagens ao ROS. 
Para instalar ela:

    pip3 install gym==0.24.1
    git clone https://github.com/alikolling/gym_stage
    cd gym_stage
    pip3 install .

Com o *gym* instalado, pode-se compilar os pacotes com o catkin.
A execução ocorre com o comando:

    roslaunch stage_controller launcher.launch

Foram escolhidos 7 alvos para testar o algoritmo. Destes, verificou-se que o método consegue alcançar 6.
