 
set remote_user=qlin1806
set remote_port=22
set remote_key="C:\Users\Quentin Lin\.ssh\id_rsa"
set remote_open_port=1995
set remote_host=gateway.qlin1806.com

set local_target_host=127.0.0.1
set local_target_port=20001


ssh -N -i %remote_key% -R %remote_open_port%:%local_target_host%:%local_target_port% %remote_host% -l %remote_user% -p %remote_port%

pause