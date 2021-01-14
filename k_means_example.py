# an example of image color segmentation.
from pyclustering.utils import draw_image_mask_segments, read_image;

from pyclustering.samples.definitions import IMAGE_SIMPLE_SAMPLES;

from pyclustering.cluster.kmeans import kmeans;
from pyclustering.cluster.center_initializer import kmeans_plusplus_initializer

# load image from the pyclustering collection.
data = read_image("4_projeta/inner_img_rgb.tif")

# set initial centers for K-Means algorithm.
amount_initial_centers = 2
initial_centers = kmeans_plusplus_initializer(data, amount_initial_centers).initialize()

# create K-Means algorithm instance.
kmeans_instance = kmeans(data, initial_centers)

# start processing.
kmeans_instance.process()

# obtain clusters that are considered as segments.
segments = kmeans_instance.get_clusters()

print(len(segments[1]))

# show image segmentation results.
clusterized = draw_image_mask_segments("4_projeta/inner_img_rgb.tif", segments)

clusterized.save('5_extrai_linhas/grupos.tif')
