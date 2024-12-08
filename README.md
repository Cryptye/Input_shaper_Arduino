# Input_shaper_Arduino

Descrição do Projeto

Este projeto consiste em um programa desenvolvido em C++ para o ambiente Arduino. O código é projetado para controlar um atuador linear, movendo uma mesa móvel à qual um pêndulo está atrelado, para ambas as situações de comandos de posição: Não-modelado e modelado a partir da técnica de input shaping por 2 impulsos.
Requisitos do Projeto

    * Placa Arduino Due
    * Motor de corrente contínua
    * Driver para controle do motor
    * Encoders de posição (2 unidades)
    * Bibliotecas NeoTimer, DueTimer, U8g2lib, Math e Arduino
    * Software VSCode com extensão PlatformIO

Configuração do Hardware

    * Conexão do motor e driver de acordo com as portas especificadas no código (INA, INB e PULSE)
    * Conexão dos encoders de acordo com as portas especificadas no código (encoder0 e encoder1 PinA/PinB)

Configuração do Software

    * Instale as bibliotecas no seu ambiente de desenvolvimento do Arduino
    * Carregue o código fornecido no VSCode
    * Configure as conexões do Hardware de acordo com as variáveis no código
    * Faça upload do código para a placa Arduino

Utilização do Programa

    * Certifique-se de que a placa Arduino está devidamente conectada.
    * Abra o Monitor Serial para interagir com o programa.
    * Envie comandos através do Monitor Serial, como descrito no código:
    * * Comandos incluem movimentos manuais, movimentação não controlada e movimentação controlada por input shaping.

Licença
Este projeto é distribuído gratuitamente. Sinta-se à vontade para usar, modificar e distribuir conforme necessário.
