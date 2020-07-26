javac -d bin/ -cp src/Common/*.java
javac -d bin/ -cp "src;src/Common/*.java;src/GameComms/*.java" src/Main/Server.java
cd bin
jar -cvf ../DroneCommandServer.jar Common/*.class GameComms/*.class Main/*.class
pause
