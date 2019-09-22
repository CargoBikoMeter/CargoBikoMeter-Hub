counts the distance traveled with a bicycle based on hub dynamo frequency and sends it via LoRaWAN to remote server

![alt text](https://github.com/CargoBikoMeter/CargoBikoMeter-Hub/blob/master/CargoBikoMeter-Demonstrator-medium-size.jpg)

This is the Cargobikometer demonstrator. The hub dynamo can be driven by hand or by a DC motor. The signal from the hub dynamo will be converted from a sine wave into a rectangle signal by a schmitt trigger. To avoid an always running hub dynamo during programming, I use a simple ESP32 based signal generator with touch sensors for switching the frequency. For fun a little motor will be driven from the hub dynamo voltage by an AC-DC converter.
