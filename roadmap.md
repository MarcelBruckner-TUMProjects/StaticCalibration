# Roadmap

### Improve mapping detection

- Increase number of elements per mapping
- Increase number of matches per element
- Check why sometimes bad mappings give low evaluation error => adjust error calculation

### Calibration

- Create recent calibration images from the current video streams of the testbed
- Do the watersheding algorithm on them
- Create new mappings
- Rerun calibration

### Detection of roadmarks

- See Google and the OpenCV docs on how to segment the white road marks from the image, should be possible for our
  purpose
- If this classic approach really does not work:
    - Train the DeepLab V3+ image segmentation model on
    - A2D2 dataset (with roadmark labels)

# Tips

### Calibration

- Use the `app/CalibrateByHand` program to visualize the calibration and to tweak the values by hand

### Watersheder

- Use only a few and possibly far apart road marks

