 
set remote_user=icplogin
set remote_port=22
set remote_key="C:\Users\Quentin Lin\.ssh\id_rsa"
set remote_open_port=20001
set remote_host=192.168.137.122

set local_target_host=127.0.0.1
set local_target_port=20001


ssh -N -i %remote_key% -R %remote_open_port%:%local_target_host%:%local_target_port% %remote_host% -l %remote_user% -p %remote_port%

pause