# Protocolo de comunicacao

O ESP32 e o STM32 trocam mensagens por UART em 115200 bps. Cada comando e enviado como texto terminado por `\r\n`.

## Comandos principais

| Comando | Origem comum | Resultado esperado |
| --- | --- | --- |
| `GET` | ESP32 | Solicita telemetria |
| `PROFILE=ESTANHO` | Web | Seleciona perfil de material |
| `SET10,400` | Web/fisico | Define setpoint de 40.0 C |
| `HEATTIME=10` | Web/fisico | Define tempo de aquecimento |
| `SOAK=10` | Web/fisico | Define tempo de patamar |
| `CONFIRM` | Web/fisico | Confirma e inicia |
| `BACK` | Web/fisico | Retorna etapa de configuracao |
| `STATE=INICIO` | Web | Volta ao inicio |
| `RESET` | Web/fisico | Limpa emergencia e volta ao estado seguro |
| `MANUAL=50` | Web | Entra em modo manual com 50% de PWM |
| `SAFETY,1` | Web | Ativa seguranca da porta |
| `SAFETY,0` | Web | Desativa retorno automatico por porta |
| `ANGLE=150` | Web | Define limite da porta em 15.0 graus |
| `CAL` | Web | Calibra MPU6050 |

## Respostas

| Prefixo | Significado |
| --- | --- |
| `DATA,...` | Linha de telemetria com estado, sensores, PWM e sincronizacao |
| `ACK,...` | Comando aceito pelo STM32 |
| `ERR,...` | Comando recusado ou falha de validacao |
| `LOG,...` | Linha usada pelo historico do dashboard |

## Rotas HTTP do ESP32

| Rota | Metodo | Descricao |
| --- | --- | --- |
| `/` | GET | Pagina web principal |
| `/api/data` | GET | Snapshot JSON do estado atual |
| `/api/cmd` | GET/POST | Entrada de comandos para o STM32 |
| `/api/export` | GET | Exportacao CSV do historico |
