@echo off
:: Ball Balancer — ferramenta principal de operacao.
:: Calibracao da tela, bola ao vivo, orientacao (gravar) e ajuste de PID em
:: tempo real (sem rebuild). Le porta/dimensoes de config.py.
::
:: Uso opcional:  calibrate.bat COM5        (porta explicita)
::                calibrate.bat COM5 115200
python "%~dp0calibrate.py" %*
