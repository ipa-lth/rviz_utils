# publish_media

These nodes load an image or a video and stream it to a image-topic. This way this media can be shown using rviz plugins, rqt-plugins or image_view.

You may want to show a __static image__ (e.g. Logo), use 

```roslaunch publish_media show_image.launch```

or a __'static' mpg video__, use

```roslaunch publish_media show_video.launch```

or __images based on a string topic lookup__, use

```roslaunch publish_media show_image_demon.launch```
