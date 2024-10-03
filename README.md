counts the distance traveled with a cargobike based on hub dynamo frequency and sends it via LoRaWAN to remote server

![alt text](https://github.com/CargoBikoMeter/CargoBikoMeter-Hub/blob/master/images/Cargobikometer-Demonstrator-20230214-1024x684.jpg)

The Cargobikometer demonstrator, a project of the Sustainable Accessible Innovations Laboratory (SAI-Lab) Berlin (https://www.sai-lab.de/index.php/de/) in the field of cargobike technologies, demonstrates the basic principle of counting the traveled distance of a cargobike. 

The hub dynamo can be driven by hand or by a DC motor. The signal from the hub dynamo will be converted from a sine wave into a rectangle signal by a schmitt trigger. To avoid an always running hub dynamo during programming, we use a simple ESP32 based signal generator with touch sensors for switching the frequency. For fun a little motor will be driven from the hub dynamo voltage by an AC-DC converter.

The Cargobikometer demonstrator is the basic technology for transferring the kilometers actually driven into the virtual CargoBikeCity game for elementary school students using IoT LoRaWAN in the future and using it in the game as an energy store for a wide variety of activities. More information about our CargoBikeCity idea can be found in the Arena of Ideas for a Smart City Berlin (https://mein.berlin.de/ideas/2023-15758/).

The circuit diagram and the structure of the circuit using a breadboard are shown below.
![alt text](https://github.com/CargoBikoMeter/CargoBikoMeter-Hub/blob/master/images/cbm-Heltec-LoRaV2_Schaltplan.png)
![alt text](https://github.com/CargoBikoMeter/CargoBikoMeter-Hub/blob/master/images/cbm-Heltec-LoRaV2_Steckplatine.png)
