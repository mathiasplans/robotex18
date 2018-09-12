# Forwards local port 2222 to remote port 12312 in an aws server. Usage: ssh -p 2222 robot@localhost
ssh -fNT -L 2222:localhost:12312 ec2-user@18.184.199.20 -i ~/robot_bridge.pem
