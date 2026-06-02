@echo off
:: PIDSimba — simulacao fisica do Ball Balancer 3RPS (motores + bola).
::
:: Roda no python do sistema (pygame/pyserial). Nao precisa do ESP-IDF: e uma
:: ferramenta de simulacao/visualizacao. No modo REAL le a telemetria do ESP32
:: pela serial; no modo SIM fecha o laco PID contra um modelo fisico.
::
:: Uso opcional:  PIDSimba.bat COM5        (porta explicita)
::                PIDSimba.bat --sim        (abre direto na simulacao fisica)
python "%~dp0PIDSimba.py" %*
