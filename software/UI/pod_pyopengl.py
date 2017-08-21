#!/usr/bin/env python

#signif reference to http://pastebin.com/k87sfiEf


import sys
import math
import signal
import time 

import math, random
import numpy as np
from numpy import linalg
from PIL import Image

#interface stuff
from PyQt4 import QtCore, QtGui, QtOpenGL
import pyqtgraph as pg

#comms stuff
import lcm
from mithl import vectorXf_t
from mithl import trigger_t
from mithl import velocity_t
from lcm_utils import *

#read yaml config information
import yaml

import logging
logger = logging.getLogger("pyassimp")
gllogger = logging.getLogger("OpenGL")
gllogger.setLevel(logging.WARNING)
logging.basicConfig(level=logging.INFO)

try:
    import OpenGL
except ImportError:
    app = QtGui.QApplication(sys.argv)
    QtGui.QMessageBox.critical(None, "PyQT with OpenGL",
            "PyOpenGL must be installed to run this code.")
    sys.exit(1)
OpenGL.ERROR_CHECKING=False
OpenGL.ERROR_LOGGING = False
#OpenGL.ERROR_ON_COPY = True
#OpenGL.FULL_LOGGING = True
from OpenGL.GL import *
from OpenGL.error import GLError
from OpenGL.GLU import *
from OpenGL.GLUT import *
from OpenGL.arrays import vbo
from OpenGL.GL import shaders


import pyassimp
from pyassimp.postprocess import *
from pyassimp.helper import *

class DefaultCamera:
    def __init__(self, w, h, fov, x=3000.0, y=-1000.0, z=500.0):
        self.clipplanenear = 1.0
        self.clipplanefar = 10000.0
        self.aspect = w/h
        self.horizontalfov = fov * math.pi/180
        self.origin = numpy.array([0.0,0,0.0])
        self.distance = 2000.
        self.pitch = 0
        self.yaw = 0
    def __str__(self):
        return "Default camera"

class PodGLWidget(QtOpenGL.QGLWidget):

    def __init__(self, parent=None, vis_config=None, vis_config_file_path="./"):
        super(PodGLWidget, self).__init__(parent)
        
        self.lastPos = QtCore.QPoint()
        self.vis_config_file_path = vis_config_file_path

        self.vis_config = vis_config;
        self.fov = 70
        self.lineOffset = 0.
        self.cameras = [DefaultCamera(self.size().width(),self.size().height(),self.fov)]
        self.models = {}
        self.global_offset = [0, 0, 0, 0, 0, 0]
        self.current_cam_index = 0
        self.setAutoBufferSwap(True)

        self.bb_min = numpy.array([0, 0, 0])
        self.bb_max = numpy.array([0, 0, 0])        

    def prepare_shaders(self):

        phong_weightCalc = """
        float phong_weightCalc(
            in vec3 light_pos, // light position
            in vec3 frag_normal // geometry normal
        ) {
            // returns vec2( ambientMult, diffuseMult )
            float n_dot_pos = max( 0.0, dot(
                frag_normal, light_pos
            ));
            return n_dot_pos;
        }
        """

        vertex = shaders.compileShader( phong_weightCalc +
        """
        uniform vec4 Global_ambient;
        uniform vec4 Light_ambient;
        uniform vec4 Light_diffuse;
        uniform vec3 Light_location;
        uniform vec4 Material_ambient;
        uniform vec4 Material_diffuse;
        attribute vec3 Vertex_position;
        attribute vec3 Vertex_normal;
        attribute vec2 texcoords;
        varying vec4 baseColor;
        varying vec2 texcoords_v;
        void main() {
            gl_Position = gl_ModelViewProjectionMatrix * vec4(
                Vertex_position, 1.0
            );
            vec3 EC_Light_location = gl_NormalMatrix * Light_location;
            float diffuse_weight = phong_weightCalc(
                normalize(EC_Light_location),
                normalize(gl_NormalMatrix * Vertex_normal)
            );
            baseColor = clamp(
            (
                // global component
                (Global_ambient * Material_ambient)
                // material's interaction with light's contribution
                // to the ambient lighting...
                + (Light_ambient * Material_diffuse)
                // material's interaction with the direct light from
                // the light.
                + (Light_diffuse * diffuse_weight)
            ), 0.0, 1.0);
            texcoords_v = texcoords;
        }""", GL_VERTEX_SHADER)

        fragment =  shaders.compileShader('''
            varying vec2 texcoords_v;
            uniform sampler2D thetexture;
            varying vec4 baseColor;
            void main() {
                gl_FragColor = baseColor*texture2D(thetexture, texcoords_v);
            }''', GL_FRAGMENT_SHADER)

        self.shader = shaders.compileProgram(vertex,fragment)
        self.set_shader_accessors( (
            'Global_ambient',
            'Light_ambient','Light_diffuse','Light_location',
            'Material_ambient','Material_diffuse', 'thetexture'
        ), (
            'Vertex_position','Vertex_normal', 'texcoords'
        ), self.shader)

    def set_shader_accessors(self, uniforms, attributes, shader):
        # add accessors to the shaders uniforms and attributes
        for uniform in uniforms:
            location = glGetUniformLocation( shader,  uniform )
            if location in (None,-1):
                logger.warning('No uniform: %s'%( uniform ))
            setattr( shader, uniform, location )

        for attribute in attributes:
            location = glGetAttribLocation( shader, attribute )
            if location in (None,-1):
                logger.warning('No attribute: %s'%( attribute ))
            setattr( shader, attribute, location )


    def prepare_gl_buffers(self, mesh):

        mesh.gl = {}

        # Fill the buffer for vertex and normals positions
        v = numpy.array(mesh.vertices, 'f')
        n = numpy.array(mesh.normals, 'f')
        if (mesh.texturecoords.any()):
            tc = numpy.array(mesh.texturecoords[0], 'f')
            tc = tc[:, 0:2]
            tc[:, 1] = 1.0 - tc[:, 1] # flip y axis
        else:
            tc = numpy.zeros((mesh.normals.shape[0], 2))
        mesh.gl["vbo"] = vbo.VBO(numpy.hstack((v,n, tc)))

        # Fill the buffer for vertex positions
        mesh.gl["faces"] = glGenBuffers(1)

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh.gl["faces"])
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, 
                    mesh.faces,
                    GL_STATIC_DRAW)
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,0)


    def TexFromPNG(self, filename):
        #http://stackoverflow.com/questions/5907613/render-a-textured-rectangle-with-pyopengl
        img = Image.open(filename)
        img_data = numpy.array(list(img.getdata()), numpy.uint8)

        texture = glGenTextures(1)
        #glPixelStorei(GL_UNPACK_ALIGNMENT,1)
        glActiveTexture(GL_TEXTURE0)
        glBindTexture(GL_TEXTURE_2D, texture)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, img.size[1], img.size[0], 0, GL_RGBA, GL_UNSIGNED_BYTE, img_data)
     #   glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP)
     #   glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
        glBindTexture(GL_TEXTURE_2D, 0)
        return texture

    def add_model(self, name, path, texture_path=None, offset=None, color=None, parent=None, postprocess = aiProcessPreset_TargetRealtime_MaxQuality):
        logger.info("Loading model " + name + ":" + path + "...")

        if postprocess:
            scene = pyassimp.load(path, postprocess)
        else:
            scene = pyassimp.load(path)
        logger.info("Done.")

        #log some statistics
        logger.info("  meshes: %d" % len(scene.meshes))
        logger.info("  total faces: %d" % sum([len(mesh.faces) for mesh in scene.meshes]))
        logger.info("  materials: %d" % len(scene.materials))
        bb_min, bb_max = get_bounding_box(scene)
        for i in range(3):
            self.bb_min[i] = min(self.bb_min[i], bb_min[i])
            self.bb_max[i] = max(self.bb_max[i], bb_max[i])

        logger.info("  bounding box:" + str(self.bb_min) + " - " + str(self.bb_max))
        logger.info("  textures: %d" % len(scene.textures))

        scene_center = [(a + b) / 2. for a, b in zip(self.bb_min, self.bb_max)]

        for index, mesh in enumerate(scene.meshes):
            self.prepare_gl_buffers(mesh)
            logger.info(" mesh %d: tex coords %d, uv components %d" % (index, len(mesh.texturecoords), len(mesh.numuvcomponents)))

        # Finally release the model
        pyassimp.release(scene)

        # and load texture
        if texture_path:
            print "TEXTURE LOADED"
            texture = self.TexFromPNG(texture_path)
        else:
            texture = None

        logger.info("Done loading model " + name + "!")

        if offset is None:
            offset = [0,0,0,0,0,0]
        if color is None:
            color = [1.0,1.0,1.0,1.0]

        self.models[name] = [scene, texture, offset, parent, color, color] #second color is defualt oclor

    def get_camera(self):
        return self.cameras[self.current_cam_index]

    def get_default_color(self, modelname):
        if modelname not in self.models.keys():
            print "Model ", modelname, " not a model we known of."
        else:
            return self.models[modelname][5]

    def set_color(self, modelname, color):
        if color is not None:
            if len(color) == 3:
                color.append(1.0)
            if len(color) != 4:
                print "Must use color with length 4."
            elif modelname not in self.models.keys():
                print "Model ", modelname, " not a model we known of."
            else:
                self.models[modelname][4] = color
                self.needRedraw = True

    def set_offset(self, modelname, offset):
        if len(offset) != 6:
            print "Must use offset with length 6 (xyzrpy"
        elif modelname is None:
            for i, x in enumerate(offset):
                if x:
                    self.global_offset[i] = x
        elif modelname not in self.models.keys():
            print "Model ", modelname, " not a model we know of."
        else:
            for i, x in enumerate(offset):
                if x:
                    self.models[modelname][2][i] = x
            self.needRedraw = True

    def set_camera_projection(self, camera = None):

        if not camera:
            camera = self.get_camera()

        znear = camera.clipplanenear
        zfar = camera.clipplanefar
        aspect = camera.aspect
        fov = camera.horizontalfov

        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()

        # Compute gl frustrum
        tangent = math.tan(fov/2.)
        h = znear * tangent
        w = h * aspect

        # params: left, right, bottom, top, near, far
        glFrustum(-w, w, -h, h, znear, zfar)
        # equivalent to:
        #gluPerspective(fov * 180/math.pi, aspect, znear, zfar)
        #glMatrixMode(GL_MODELVIEW)
        #glLoadIdentity()

    def cycle_cameras(self):
        if not self.cameras:
            logger.info("No camera in the scene")
            return None
        self.current_cam_index = (self.current_cam_index + 1) % len(self.cameras)
        self.current_cam = self.cameras[self.current_cam_index]
        self.set_camera(self.current_cam)
        logger.info("Switched to camera <%s>" % self.current_cam)

    def set_camera(self, camera):
        self.set_camera_projection(camera)

        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

        cam = [camera.distance, 0.0, 0.0]
        gluLookAt(cam[0], cam[1], cam[2],
                   0,  0,  0,
                       0,      1,       0)
        glRotate(camera.pitch, 0.0, 0.0, 1.0)
        glRotate(camera.yaw, .0, 1.0, .0)

        glTranslate(-camera.origin[0], -camera.origin[1], -camera.origin[2])
        
    def render(self, wireframe = False, twosided = False):

        glEnable(GL_DEPTH_TEST)
        glEnable(GL_BLEND)
        glEnable(GL_LINE_SMOOTH)
        glEnable(GL_POLYGON_SMOOTH)
        glEnable(GL_POINT_SMOOTH)
        glEnable(GL_MULTISAMPLE)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glDepthFunc(GL_LEQUAL)


        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE if wireframe else GL_FILL)
        glDisable(GL_CULL_FACE) if twosided else glEnable(GL_CULL_FACE)

        shader = self.shader

        glUseProgram(shader)
        glUniform4f( shader.Global_ambient, .7,.7,.7, .8 )
        glUniform4f( shader.Light_ambient, .6,.6,.6, 1.0 )
        glUniform4f( shader.Light_diffuse, .6,.6,.6,.6 )
        glUniform3f( shader.Light_location, 2,1,1 )

        # camera movement
        self.set_camera(self.get_camera())

        # perform global offset
        glPushMatrix()
        glTranslate(self.global_offset[0], self.global_offset[1], self.global_offset[2])
        glRotate(self.global_offset[4], 0.0,1.0,0.0)
        glRotate(self.global_offset[5], 0.0,0.0,1.0)
        glRotate(self.global_offset[3], 1.0,0.0,0.0)

        for model_name in self.models.keys():
            model = self.models[model_name]

            # get parent transforms to apply
            transform_queue = [model[2]]
            curr = model[3]
            while curr is not None and curr in self.models.keys():
                curr = self.models[curr]
                transform_queue.append(curr[2])
                curr = curr[3]
            transform_queue.reverse()
            
            glPushMatrix()
            for transform in transform_queue:
                glTranslate(transform[0],transform[1],transform[2])
                glRotate(transform[3], 1.0,0.0,0.0)
                glRotate(transform[4], 0.0,1.0,0.0)
                glRotate(transform[5], 0.0,0.0,1.0)
            self.recursive_render(model[0].rootnode, model[1], shader, model[4])
            glPopMatrix()



        # end camera movement
        glPopMatrix()
        
        glBindTexture(GL_TEXTURE_2D, self.dummy_texture)
        
        glUniform4f( shader.Material_diffuse, *[1,1,1,1] )
        glUniform4f( shader.Material_ambient, *[1,1,1,1] )
        # Draw in lines on the ground every 5 meters, which move by at our flying velocity
        spacing = 10000.
        for i in range(-10, 10):
            glBegin(GL_QUADS)
            glLineWidth(100)
            glColor4f(1.0, 1.0, 0.0, 1.0)
            glVertex3f(-1000.0, -500., self.lineOffset%spacing+spacing*i-50)
            glVertex3f(1000.0, -500., self.lineOffset%spacing+spacing*i-50)
            glVertex3f(1000.0, -500., self.lineOffset%spacing+spacing*i+50)
            glVertex3f(-1000.0, -500., self.lineOffset%spacing+spacing*i+50)

            glVertex3f(-1000.0, -500., self.lineOffset%spacing+spacing*i+50)
            glVertex3f(1000.0, -500., self.lineOffset%spacing+spacing*i+50)
            glVertex3f(1000.0, -500., self.lineOffset%spacing+spacing*i-50)
            glVertex3f(-1000.0, -500., self.lineOffset%spacing+spacing*i-50)

            glEnd()

        glBindTexture(GL_TEXTURE_2D, 0)


        glUseProgram( 0 )

    def recursive_render(self, node, texture, shader, color):
        """ Main recursive rendering method.
        """

        # save model matrix and apply node transformation
        glPushMatrix()
        m = node.transformation.transpose() # OpenGL row major
        glMultMatrixf(m)

        for mesh in node.meshes:

            stride = 32 # 8 * 4 bytes

            #diffuse = mesh.material.properties["diffuse"]
            #if len(diffuse) == 3: diffuse.append(1.0)
            #ambient = mesh.material.properties["ambient"]
            #if len(ambient) == 3: ambient.append(1.0)

            glUniform4f( shader.Material_diffuse, *color )
            glUniform4f( shader.Material_ambient, *color )
            # has been bound to texture #0 (i.e. texture #1... confusing)
            if texture:
                glBindTexture(GL_TEXTURE_2D, texture)
                glUniform1i(shader.thetexture,0)
            else:
                glBindTexture(GL_TEXTURE_2D, self.dummy_texture)
                glUniform1i(shader.thetexture,0)

            vbo = mesh.gl["vbo"]
            vbo.bind()

            glEnableVertexAttribArray( shader.Vertex_position )
            glEnableVertexAttribArray( shader.Vertex_normal )
            glEnableVertexAttribArray( shader.texcoords )

            glVertexAttribPointer(
                shader.Vertex_position,
                3, GL_FLOAT,False, stride, vbo
            )

            glVertexAttribPointer(
                shader.Vertex_normal,
                3, GL_FLOAT,False, stride, vbo+12
            )

            glVertexAttribPointer(
                shader.texcoords,
                2, GL_FLOAT,False, stride, vbo+24
            )


            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh.gl["faces"])
            glDrawElements(GL_TRIANGLES, len(mesh.faces) * 3, GL_UNSIGNED_INT, None)


            vbo.unbind()
            glDisableVertexAttribArray( shader.Vertex_position )

            glDisableVertexAttribArray( shader.Vertex_normal )

            glDisableVertexAttribArray( shader.texcoords )

            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0)
            glBindTexture(GL_TEXTURE_2D, 0)

        for child in node.children:
            self.recursive_render(child, texture, shader, color)

        glPopMatrix()

    def minimumSizeHint(self):
        return QtCore.QSize(640, 480)

    def sizeHint(self):
        return QtCore.QSize(1027, 768)

    def initializeGL(self):
        glEnable(GL_TEXTURE_2D)
        glEnable(GL_COLOR_MATERIAL)

        # make a white dummy texture
        self.dummy_texture = glGenTextures(1)
        glActiveTexture(GL_TEXTURE0)
        glBindTexture(GL_TEXTURE_2D, self.dummy_texture)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, numpy.array([255, 255, 255, 255], numpy.uint8))
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
        glBindTexture(GL_TEXTURE_2D, 0)

        self.prepare_shaders()

        for pod_part_name in self.vis_config.keys():
            pod_part = self.vis_config[pod_part_name]
            texturepath = None
            if "texture" in pod_part.keys():
                texturepath = os.path.join(self.vis_config_file_path, pod_part["texture"])
            offset = [0,0,0,0,0,0]
            if "offset" in pod_part.keys():
                offset = pod_part["offset"]
            parent = None
            if "parent" in pod_part.keys():
                parent = pod_part["parent"]
            color = []
            if "color" in pod_part.keys():
                color = pod_part["color"]

            self.add_model(name = pod_part_name,
                           path= os.path.join(self.vis_config_file_path, pod_part["mesh"]),
                           texture_path=texturepath, 
                           color=color,
                           offset=offset, parent=parent)

        #self.cameras[0].origin = (self.bb_min + self.bb_max) / 2.
        #print "Put camera origin at ", self.cameras[0].origin
        self.cycle_cameras()

    def paintGL(self):
        #GL.glMatrixMode( GL.GL_PROJECTION )
        #GL.glLoadIdentity()

        # have this persist frame-to-frame so we can do our
        # relative motion in the key press events
        # to be replacedwith much smarter in a bit
        #glMatrixMode( GL_MODELVIEW );
        #glLoadIdentity();

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        self.render()

    def resizeGL(self, width, height):
        glViewport(0, 0, width, height)
        for camera in self.cameras:
            camera.aspect = width/height
        self.set_camera_projection()

    def mousePressEvent(self, event):
        self.lastPos = event.pos()

    def mouseMoveEvent(self, event):
        dx = event.x() - self.lastPos.x()
        dy = event.y() - self.lastPos.y()

        if event.buttons() & QtCore.Qt.LeftButton:
            look_speed = .2
            self.get_camera().pitch -= dy
            self.get_camera().yaw += dx

        if event.buttons() & QtCore.Qt.RightButton:
            self.get_camera().distance += 10*dy

        self.lastPos = event.pos()

        self.update()

    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key_W:
            fwd = 100
        elif event.key() == QtCore.Qt.Key_S:
            fwd = -100
        else:
            fwd = 0

        if event.key() == QtCore.Qt.Key_A:
            strafe = 100
        elif event.key() == QtCore.Qt.Key_D:
            strafe = -100
        else:
            strafe = 0

        if abs(fwd) or abs(strafe):
            m = glGetDoublev(GL_MODELVIEW_MATRIX).flatten()
            glTranslate(fwd*m[2],fwd*m[6],fwd*m[10])
            glTranslate(strafe*m[0],strafe*m[4],strafe*m[8])

        self.update()

if __name__ == '__main__':
    # hook up interrupt signal
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    vis_config_file = "../models/pod.yaml"
    vis_config_file_path = os.path.abspath(os.path.dirname(vis_config_file))

    with open(vis_config_file, 'r') as f:
        vis_config = yaml.load(f)
    
    lc = create_lcm()

    app = QtGui.QApplication(sys.argv)
    window = PodGLWidget(vis_config=vis_config, vis_config_file_path=vis_config_file_path)
    window.show()

    start_lcm(lc)

    sys.exit(app.exec_())
