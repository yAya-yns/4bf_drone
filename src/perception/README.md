# Camera notes: 
- You need to reboot the Jetson after connecting the camera to Jetson.
- To view camera images, in terminal, run: 
```
nvgstcapture-1.0
```
## Camera Reddish correction: 
`https://www.waveshare.com/wiki/IMX219-160_Camera#:~:text=If%20the%20camera%20shooting%20effect%20is%20reddish%2C%20you%20can%20follow%20the%20steps%20below%3A`
## Setup: 
`https://github.com/NVIDIA-AI-IOT/jetcam`
## Two ways of receiving images for python
- Option 1:
    ```
    image = camera.read()
    ```
- Option 2: 
    ```
    camera.read()
    image = camera.value
    ```