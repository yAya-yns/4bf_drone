import ipywidgets
from IPython.display import display
from jetcam.utils import bgr8_to_jpeg

image_widget = ipywidgets.Image(format='jpeg')

image_widget.value = bgr8_to_jpeg(image)

display(image_widget)


# camera.running = True

# def update_image(change):
#     image = change['new']
#     image_widget.value = bgr8_to_jpeg(image)
    
# camera.observe(update_image, names='value')