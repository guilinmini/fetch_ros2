#! /usr/bin/env python
# -*- coding: utf-8 -*-

import os

for i in xrange(0,12):
    # 运行ar_tracker_alvar库的createMarker节点，根据给定数字生成其对应的ALVAR码PNG图片
    os.system("rosrun ar_track_alvar createMarker {0}".format(i))
    fn = "MarkerData_{0}.png".format(i)
    # 运行ImageMagick工具给ALVAR二维码周围加上白边，这样有助于提高识别效果
    os.system("convert {0} -bordercolor white -border 100x100 {0}".format(fn)) 

    # 生成一个包含ALVAR二维码图像作为纹理的材质脚本
    with open("product_{0}.material".format(i), 'w') as f:
        f.write("""
material product_%d {
    receive_shadows on
        technique {
        pass {
            ambient 1.0 1.0 1.0 1.0
            diffuse 1.0 1.0 1.0 1.0
            specular 0.5 0.5 0.5 1.0
            lighting on
            shading gouraud
            texture_unit { texture MarkerData_%d.png }
        }
    }
}
        """ % (i,i))