from PIL import Image
import io


class RosImage:
    def __init__(self, image_data) -> None:
        #TODO: Resize should be removed!
        self.img = Image.open(io.BytesIO(image_data)).resize([1920, 1080])

    def save(self, filename: str):
        self.img.save(filename)
