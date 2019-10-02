import vtk
import math
import numpy as np

class vtpDrawScene :
    
    def SetQuatOrientation( self, quaternion ):
        if not self.iniOk :
            raise Exception("vtpDrawScene not initialized. Call initScene() first")

        # Convert quat to the rotation matrix
        mtxRot =  [[0,0,0],[0,0,0],[0,0,0]]
        vtk.vtkMath().QuaternionToMatrix3x3(quaternion, mtxRot)
        
        # Rotation: convert 3x3 to 4x4 matrix
        mtxTr2 = vtk.vtkMatrix4x4() # identity mtx
        for i in range(3):
            for j in range(3) :
                mtxTr2.SetElement(i, j, mtxRot[i][j])
        
        self.modelActor.SetUserMatrix(mtxTr2)
        self.renWin.Render()

    def initScene(self): 
           
        ren = vtk.vtkRenderer()
        self.renWin = vtk.vtkRenderWindow()
        self.renWin.AddRenderer(ren)
        
        filename = "teapot.vtk"
        reader = vtk.vtkPolyDataReader()
 
        reader.SetFileName(filename)
        reader.Update()
        
        # make mapper for the data
        modelMapper = vtk.vtkPolyDataMapper()
        modelMapper.SetInputData(reader.GetOutput())
        
        # create actor, set its mapper
        self.modelActor = vtk.vtkActor()
        self.modelActor.SetMapper(modelMapper)
        
        # Add the actor to the renderer, set the background and size.
        ren.AddActor(self.modelActor)
        ren.SetBackground(0.1, 0.2, 0.4)
        self.renWin.SetSize(800, 800)
        self.modelActor.SetPosition(0.0,0,0)
        camera = vtk.vtkCamera()
        camera.SetPosition(-10, 0, 0);
        camera.SetFocalPoint(0, 0, 0);
        camera.SetRoll(90)
        ren.SetActiveCamera(camera)
        self.renWin.Render()
        
        self.iniOk = True
        
    
    def __init__(self):
        self.iniOk = False

    def __del__(self):
        self.renWin.Finalize()

if __name__ == "__main__":
    # create quaternion
    # rotation origin
    theta = math.pi * -0.5
    # x y z coodinates of the rotation vector
    rot_ax = math.sin(theta/2) * np.array( [1.0, 1.0, .0])
    qtr_arr = np.insert(rot_ax, 0, math.cos(theta/2), axis=0)
#    qtr_arr = [1.0, 0.0, 0.0, 0.0] # unity quat
    # print qtr_arr
    qtr = vtk.vtkQuaterniond( qtr_arr ) 
    #print qtr.Normalized()
    
    scene = vtpDrawScene()
    scene.initScene()
    scene.SetQuatOrientation(qtr)
    raw_input("Press Enter to exit")
    
    del scene


