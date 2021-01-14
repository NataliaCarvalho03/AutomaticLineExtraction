# an example of image color segmentation.
from pyclustering.utils import draw_image_mask_segments, read_image;

from pyclustering.samples.definitions import IMAGE_SIMPLE_SAMPLES;

from pyclustering.cluster.kmeans import kmeans;

from pyclustering.cluster.center_initializer import kmeans_plusplus_initializer

# load image from the pyclustering collection.
data = read_image(IMAGE_SIMPLE_SAMPLES.IMAGE_SIMPLE_BEACH)

# set initial centers for K-Means algorithm.
amount_initial_centers = 3
initial_centers = kmeans_plusplus_initializer(data, amount_initial_centers).initialize()
#start_centers = [ [153, 217, 234, 128], [0, 162, 232, 128], [34, 177, 76, 128], [255, 242, 0, 128] ]

# create K-Means algorithm instance.
kmeans_instance = kmeans(data, initial_centers)

# start processing.
kmeans_instance.process()

# obtain clusters that are considered as segments.
segments = kmeans_instance.get_clusters()

# show image segmentation results.
draw_image_mask_segments(IMAGE_SIMPLE_SAMPLES.IMAGE_SIMPLE_BEACH, segments)