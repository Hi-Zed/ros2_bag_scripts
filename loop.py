import extractPCD
import os

# if __name__ == '__main__':
#     for n in range(8):
#         src = '/run/media/gianluca/airlab_ds/carmagnola_20230608/full_bags/test_bag_0'+str(n+1)
#         print(src)
#         dest = '/run/media/gianluca/airlab_ds/carmagnola_20230608/pcd/test_bag_0'+str(n+1)+'_pcd'
#         print(dest)
#         if not os.path.exists(dest):
#             os.makedirs(dest)
#         extractPCD.extract_pcd(src, dest)

if __name__ == '__main__':
    for n in range(5):
        src = '/run/media/gianluca/airlab_ds/carmagnola_20230608/full_bags/test_bag_0'+str(n+1)
        print(src)
        dest = '/run/media/gianluca/airlab_ds/carmagnola_20230608/pcd/test_bag_0'+str(n+1)+'_img'
        print(dest)
        if not os.path.exists(dest):
            os.makedirs(dest)
        extractPCD.extract_images(src, dest)

# if __name__ == '__main__':
#     for n in range(3,13):
#         src = '/run/media/gianluca/airlab_ds/carmagnola_20230608/oak_bags/oak_bag_{:02d}'.format(n)
#         print(src)
#         dest = '/run/media/gianluca/airlab_ds/carmagnola_20230608/pcd/oak_bag_{:02d}_img'.format(n)
#         print(dest)
#         if not os.path.exists(dest):
#             os.makedirs(dest)
#         extractPCD.extract_images(src, dest)