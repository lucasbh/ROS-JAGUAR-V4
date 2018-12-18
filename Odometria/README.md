**Procurar e baixar o manual do GPS e IMU**
	Os manuais do GPS e IMU foram encontrados nos github do Jaguar:
	https://github.com/afonsohfontes/Jaguar/blob/master/Componentes%20Jaguar/9Dof%20Razor/MPU9250REV1.0.pdf 
	https://static.garmincdn.com/pumac/GPS_18x_Tech_Specs.pdf

**Saber onde conectar no Jaguar**
	As conexões do GPS e IMU no Jaguar podem ser vistas na imagem abaixo representadas pelos conectores azul e amarelo.
  
	[img]
	
**Saber como acessar informações pelo Jaguar e validar o correto funcionamento dos sensores e placas**
	Para testar o funcionamento do GPS e IMU foi utilizado o protocolo Telnet por possibilitar o acesso às informações de cada porta da placa. 
	O formato para a utilização do protocolo telnet é:
		telnet ip porta
	No terminal deve-se entrar com o comando: 
		telnet 192.168.0.61 10001 (para o IMU) 
		telnet 192.168.0.61 10002 (para o GPS)
	O GPS retorna diversas strings com diferentes leituras, o projeto requer a leitura da localização, ou seja, latitude e longitude. A leitura do manual foi essencial para a determinar a utilização da string GPRMC. Para validar se a resposta do GPS está correta foi utilizado o site: https://rl.se/gprmc. 
	O IMU é composto por um giroscópio, acelerômetro e magnetômetro. A resposta desejada é a leitura do giroscópio para determinar a orientação do Jaguar. Foi realizado um teste de movimento e verificou-se que a única resposta gerada era de aceleração nos eixos X, Y e Z.
	É necessária pesquisa quanto a configuração de leitura do IMU para adicionar a leitura do giroscópio na resposta.

**Visualizar dados do GPS e IMU via ROS**
	A resposta do GPS é dada por string, por isso, para separar as informações a fim de encontrar os valores para longitude e latitude, criou-se dois códigos que utiliza o conceito de parse. Para isso, foi utilizada a biblioteca Pynmea 2. Os códigos devem ser utilizados na ordem abaixo:
	Em um terminal entrar com: roscore
	Em outro terminal entrar com: python ‘arquivo ros-tutorial.py’
	Em outro terminal entrar com: python ‘arquivo ros-listener2.py’

**Planejamento de trajetória**
	A latitude e longitude são expressas em graus, minutos e segundos. Para realizar o cálculo da trajetória, utiliza-se uma relação de conversão para km dada por:
	Comprimento² = DLA² + DLO²
	Onde:
		DLA = Diferença de latitude
		DLO = Diferença de longitude
	Essa relação deve ser incluída no código para o cálculo da distância entre os dois pontos desejados. 


**Codigos python para os testes**
	Os codigos **ros-listener2.py** e **ros-tutorial.py** são o listener e o publisher, respectivamente. O publisher lê os dados do GPS e os publica em um Nó, enquanto o listener se inscreve na publicação e faz a tradução dos dados do GPS para leitura das coordenadas. 
		
**Trabalhos a serem realizados**

Mudar a comunicação de Telnet para sockets (melhor) com o objetivo de testar se os dados do GPS são atualizados e se os dados do IMU são informados por completo.
Estudar odometria e implementar lógica de movimentação do Jaguar por inserção de coordenadas alvo, integrando giroscópio e bússola do IMU com coordenadas obtidas pelo GPS.
