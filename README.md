<h1>Voice Controlled Robot</h1>
In this project, we built a system that can recognize, save and control the robot through periodic sounds like
A, U, O and together with Android app to observe the results.
<br />
<br />
Our team consists of three members so we split the works; while I designed the custom PCB boards, wrote the microcontroller firmware and BLE communication between the Android app and the microcontroller, my friends implemented algorithms (FFT, MFCC and computing DCT) so that the MCU can calculate frequency spectrums, MFCC filter banks to get the characteristics for each letter. The result was a surprisingly functional voice/tone recognition robot.
<br />
<br />
<br />

<h2>Images</h2>

| PCB Layout  | Schematic |
| ------------- | ------------- |
| <img src="https://github.com/KizEvo/voice-controlled-robot/assets/104358167/846650fb-880c-4105-885f-12b82169eb53" width="400">  | <img src="https://github.com/KizEvo/voice-controlled-robot/assets/104358167/1e474fe2-1928-4ff3-9be4-cb4081b678ae" width="400">  |

| Android App  | Android App |
| ------------- | ------------- |
| <img src="https://github.com/KizEvo/voice-controlled-robot/assets/104358167/bbc47cf0-e65d-4554-aa2b-0d6d5992b6d0" width="400">  | <img src="https://github.com/KizEvo/voice-controlled-robot/assets/104358167/9386c14d-dad9-480e-9797-dec5bc92a553" width="400">  |

<h2>Demo Video</h2>

[![Watch the demo](https://img.youtube.com/vi/xk0MdGKy55M/maxresdefault.jpg)](https://www.youtube.com/watch?v=xk0MdGKy55M)

<h2>Android App Tech</h2>
<ul>
<li>React Native</li>
<li>Android Studio</li>
<li>JavaScript/TypeScript</li>
<br />
         
Click here for [Android app repository](https://github.com/KizEvo/wireless-robot-control-app)
</ul>
<h2>Hardware</h2>
<ul>
<li>STM32F411CEU6</li>
<li>INMP441 I2S Microphone</li>
<li>L298N</li>
<li>HM-10 Bluetooth</li>
</ul>
          
