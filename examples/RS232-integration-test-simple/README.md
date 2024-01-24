# Simple Integration Tests Example (RS232 Interface)
Digital Twin Prototypes (DTP) need emulators of sensors and actuators to replace the hardware of a physical twin (PT). Learn
more about DTPs in my [preprint on Techrxiv](https://www.techrxiv.org/articles/preprint/Continuous_Integration_Testing_of_Embedded_Software_with_Digital_Twin_Prototypes/14770983).

### Why should I use an emulator instead of tools such as Mockito?
By using tools such as Mockito, you test your code against a mockup, but not against the interfaces used by the real system.
With Docker and Socat you can proxy a serial interface on TCP/IP and test a "real" serial connection.

# How to start the example
1. Install Docker and docker-compose
2. Clone the project.

`git clone https://github.com/cau-se/DigitalTwinPrototypes`

3. Go to the main folder

`cd \path\to\simple-integration-testing-example`

4. Execute docker-compose

`docker-compose up`

### How to execute the tests
1. Go to the test folder in simple-driver

`cd \path\to\simple-integration-testing-example\simple-driver\tests`

2. Execute docker-compose

`docker-compose up`