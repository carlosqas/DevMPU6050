Firmware de teste do MPU6050 que usa multithread mas nenhum design pattern.

--- Protocolo de comunicação ---

01) Configuração das sensibilidades

Configurar sensibilidade do tombamento:
[90], [01], [MinSamples], [StartThreshold]

Configurar sensibilidade do movimento:
[90], [02], [MinSamples], [StartThreshold * 100]

Configurar sensibilidade da parada:
[90], [03], [MinSamples], [StartThreshold * 100]

Configurar sensibilidade do tamper:
[90], [03], [MinSamples], [TamperTime (segundos)]

02) Configuração do Debug

Ativar/Desativa YPR:
[80], [01], [Controle ( 0 = desativa, 1 = ativa)]

Ativar/Desativa Accel:
[80], [02], [Controle ( 0 = desativa, 1 = ativa)]

Ativar/Desativa Gyro:
[80], [03], [Controle ( 0 = desativa, 1 = ativa)]

Ativar/Desativa Device Status:
[80], [04], [Controle ( 0 = desativa, 1 = ativa)]

Ativar/Desativa Memory Usage:
[80], [05], [Controle ( 0 = desativa, 1 = ativa)]

Ativar/Desativa Temperatura:
[80], [06]. [Controle (0 = desativa, 1 = ativa)]