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
        
        # three transforms:
        # 1. move the object so the rotation center is in the coord center  
        tr = vtk.vtkTransform()
        origin = np.array(self.modelActor.GetOrigin())
        position = np.array(self.modelActor.GetPosition())
        trans = origin+position 
    #    trans = origin
        tr.Translate(-trans)
        mtxTr1 = tr.GetMatrix()
        
        # 2. rotate around coord center using mtxTr2
        mtxTr12 = vtk.vtkMatrix4x4()
        vtk.vtkMatrix4x4().Multiply4x4 (mtxTr2, mtxTr1, mtxTr12)
        
        ## 3. move the object back
        tr = vtk.vtkTransform()
        tr.Translate(trans)
        mtxTr3 = tr.GetMatrix()
        mtxTr123 = vtk.vtkMatrix4x4()
        vtk.vtkMatrix4x4().Multiply4x4 (mtxTr3, mtxTr12, mtxTr123)
        
        tr = vtk.vtkTransform()
        tr.PreMultiply()  
    #    tr.PostMultiply()  
        tr.Concatenate(mtxTr123) 
        self.modelActor.SetUserTransform(tr)
        
        self.renWin.Render()

    def initScene(self): 
           
        ren = vtk.vtkRenderer()
        self.renWin = vtk.vtkRenderWindow()
        self.renWin.AddRenderer(ren)
        
        # Read the object data
#        filename = "hat_skull.vtp"
#        reader = vtk.vtkXMLPolyDataReader()

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
        
        # Our object has huge y-offset in the data
        # Get center of the bounding box
        origin = self.modelActor.GetCenter() 
        # Set rotation center
        self.modelActor.SetOrigin(origin) 
        
        ren.SetBackground(0.1, 0.2, 0.4)
        self.renWin.SetSize(800, 800)
        self.modelActor.SetPosition(0.0,0,0)
#        self.modelActor.SetPosition(0,-0.1,-1)
        # A hack to fix BNO and VTK axes mismatch
        camera =vtk.vtkCamera()
        camera.SetPosition(0, 100,0);
        camera.SetFocalPoint(0, 0, 0);
        ren.SetActiveCamera(camera)
        # end of hack
        ren.ResetCamera()
        self.renWin.Render()
        self.iniOk = True
        
    
    def __init__(self):
        self.iniOk = False

    def __del__(self):
        self.renWin.Finalize()

if __name__ == "__main__":
    # create quaternion
    # rotation origin
    theta = math.pi * -0.0
    # x y z coodinates of the rotation vector
    rot_ax = math.sin(theta/2) * np.array( [1.0, 1.0, .0])
    qtr_arr = np.insert(rot_ax, 0, math.cos(theta/2), axis=0)
    # print qtr_arr
    qtr = vtk.vtkQuaterniond( qtr_arr ) 
    #print qtr.Normalized()
    
    scene = vtpDrawScene()
    scene.initScene()
    scene.SetQuatOrientation(qtr)
    raw_input("Press Enter to exit")
    
    del scene


