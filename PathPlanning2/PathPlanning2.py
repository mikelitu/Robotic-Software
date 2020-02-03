import os
import unittest
import numpy as np
import vtk, qt, ctk, slicer, random
import SimpleITK as sitk
import slicer.util as su
from slicer.ScriptedLoadableModule import *
import logging

#
# PathPlanning2
#

class PathPlanning2(ScriptedLoadableModule):
  """Uses ScriptedLoadableModule base class, available at:
  https://github.com/Slicer/Slicer/blob/master/Base/Python/slicer/ScriptedLoadableModule.py
  """

  def __init__(self, parent):
    ScriptedLoadableModule.__init__(self, parent)
    self.parent.title = "PathPlanning2" # TODO make this more human readable by adding spaces
    self.parent.categories = ["Examples"]
    self.parent.dependencies = []
    self.parent.contributors = ["John Doe (AnyWare Corp.)"] # replace with "Firstname Lastname (Organization)"
    self.parent.helpText = """
This is an example of scripted loadable module bundled in an extension.
It performs a simple thresholding on the input volume and optionally captures a screenshot.
"""
    self.parent.helpText += self.getDefaultModuleDocumentationLink()
    self.parent.acknowledgementText = """
This file was originally developed by Jean-Christophe Fillion-Robin, Kitware Inc.
and Steve Pieper, Isomics, Inc. and was partially funded by NIH grant 3P41RR013218-12S1.
""" # replace with organization, grant and thanks.

#
# PathPlanning2Widget
#

class PathPlanning2Widget(ScriptedLoadableModuleWidget):
  """Uses ScriptedLoadableModuleWidget base class, available at:
  https://github.com/Slicer/Slicer/blob/master/Base/Python/slicer/ScriptedLoadableModule.py
  """

  def setup(self):
    ScriptedLoadableModuleWidget.setup(self)

    # Instantiate and connect widgets ...

    #
    # Parameters Area
    #
    parametersCollapsibleButton = ctk.ctkCollapsibleButton()
    parametersCollapsibleButton.text = "Parameters"
    self.layout.addWidget(parametersCollapsibleButton)

    # Layout within the dummy collapsible button
    parametersFormLayout = qt.QFormLayout(parametersCollapsibleButton)

    #
    # input volume selector
    #
    self.inputImageSelector = slicer.qMRMLNodeComboBox()
    self.inputImageSelector.nodeTypes = ["vtkMRMLLabelMapVolumeNode"]
    self.inputImageSelector.selectNodeUponCreation = True
    self.inputImageSelector.addEnabled = False
    self.inputImageSelector.removeEnabled = False
    self.inputImageSelector.noneEnabled = False
    self.inputImageSelector.showHidden = False
    self.inputImageSelector.showChildNodeTypes = False
    self.inputImageSelector.setMRMLScene(slicer.mrmlScene)
    self.inputImageSelector.setToolTip("Pick the input to the algorithm.")
    parametersFormLayout.addRow("Input Label Map: ", self.inputImageSelector)

    #
    # input entry points
    #
    self.inputEntrySelector = slicer.qMRMLNodeComboBox()
    self.inputEntrySelector.nodeTypes = ["vtkMRMLMarkupsFiducialNode"]
    self.inputEntrySelector.selectNodeUponCreation = True
    self.inputEntrySelector.addEnabled = False
    self.inputEntrySelector.removeEnabled = False
    self.inputEntrySelector.noneEnabled = False
    self.inputEntrySelector.showHidden = False
    self.inputEntrySelector.showChildNodeTypes = False
    self.inputEntrySelector.setMRMLScene(slicer.mrmlScene)
    self.inputEntrySelector.setToolTip("Pick the input entry fiducials to the algorithm.")
    parametersFormLayout.addRow("Input entry points: ", self.inputEntrySelector)

    #
    # input target points
    #
    self.inputTargetSelector = slicer.qMRMLNodeComboBox()
    self.inputTargetSelector.nodeTypes = ["vtkMRMLMarkupsFiducialNode"]
    self.inputTargetSelector.selectNodeUponCreation = True
    self.inputTargetSelector.addEnabled = False
    self.inputTargetSelector.removeEnabled = False
    self.inputTargetSelector.noneEnabled = False
    self.inputTargetSelector.showHidden = False
    self.inputTargetSelector.showChildNodeTypes = False
    self.inputTargetSelector.setMRMLScene(slicer.mrmlScene)
    self.inputTargetSelector.setToolTip("Pick the input target fiducials to the algorithm.")
    parametersFormLayout.addRow("Input target points: ", self.inputTargetSelector)

    #
    # check box to trigger taking screen shots for later use in tutorials
    #
    self.enableScreenshotsFlagCheckBox = qt.QCheckBox()
    self.enableScreenshotsFlagCheckBox.checked = 0
    self.enableScreenshotsFlagCheckBox.setToolTip("If checked, take screen shots for tutorials. Use Save Data to write them to disk.")
    parametersFormLayout.addRow("Enable Screenshots", self.enableScreenshotsFlagCheckBox)

    #
    # Apply Button
    #
    self.applyButton = qt.QPushButton("Apply")
    self.applyButton.toolTip = "Run the algorithm."
    self.applyButton.enabled = False
    parametersFormLayout.addRow(self.applyButton)

    # connections
    self.applyButton.connect('clicked(bool)', self.onApplyButton)
    self.inputImageSelector.connect("currentNodeChanged(vtkMRMLNode*)", self.onSelect)

    # Add vertical spacer
    self.layout.addStretch(1)

    # Refresh Apply button state
    self.onSelect()

  def cleanup(self):
    pass

  def onSelect(self):
    self.applyButton.enabled = self.inputImageSelector.currentNode() and self.inputEntrySelector.currentNode() and self.inputTargetSelector.currentNode()

  def onApplyButton(self):
    logic = PathPlanning2Logic()
    enableScreenshotsFlag = self.enableScreenshotsFlagCheckBox.checked
    logic.run(self.inputImageSelector.currentNode(), self.inputEntrySelector.currentNode(),
              self.inputTargetSelector.currentNode(), enableScreenshotsFlag)


#
# PathPlanning2Logic
#

class PathPlanning2Logic(ScriptedLoadableModuleLogic):
  """This class should implement all the actual
  computation done by your module.  The interface
  should be such that other python code can import
  this class and make use of the functionality without
  requiring an instance of the Widget.
  Uses ScriptedLoadableModuleLogic base class, available at:
  https://github.com/Slicer/Slicer/blob/master/Base/Python/slicer/ScriptedLoadableModule.py
  """

  def hasImageData(self,volumeNode):
    """This is an example logic method that
    returns true if the passed in volume
    node has valid image data
    """
    if not volumeNode:
      logging.debug('hasImageData failed: no volume node')
      return False
    if volumeNode.GetImageData() is None:
      logging.debug('hasImageData failed: no image data in volume node')
      return False
    return True

  def isValidInputOutputData(self, inputVolumeNode, entryFiducialsNode, targetFiducialNode):
    """Validates if the output is not the same as input
    """
    #Validate the inputVolumeNode
    if not inputVolumeNode:
      logging.debug('isValidInputOutputData failed: no input volume node defined')
      return False
    #Validate the entryFiducialsNode
    if not entryFiducialsNode:
      logging.debug('is ValidInputOutputData failed: no input entry fiducials node defined')
      return False
    #Validate the targetFiducialNode
    if not targetFiducialNode:
      logging.debug('is ValidInputOutputData failed: no input target fiducials node defined')
      return False
    #Validate that the entry and the target fiducials node are different
    if entryFiducialsNode.GetID()==targetFiducialNode.GetID():
      logging.debug('is ValidInputOutputData failed: the entry and the target fiducials node are the same')
      return False
    return True

  def run(self, inputVolume, entries, targets, enableScreenshots=0):
    """
    Run the actual algorithm
    """

    logging.info('Processing started')

    #Check that the input data is valid
    self.isValidInputOutputData(inputVolume, entries, targets)

    # Run the class IntersectWith()
    intersect = IntersectWith()
    intersect.run(inputVolume, entries, targets)
    danielsson = ComputeDistanceImageFromLabelMap()
    danielsson.Execute(inputVolume)

    # Capture screenshot
    if enableScreenshots:
      self.takeScreenshot('PathPlanning2Test-Start','MyScreenshot',-1)

    logging.info('Processing completed')

    return True

class MarchingCubes():
  def run(self, inputImage):
    """ Class to run the complete setup for the vtk.vtkMarchingCubes
    to generate the final mesh for the structure to avoid"""
    mc = vtk.vtkMarchingCubes() #Define the filter
    mc.SetInputData(inputImage.GetImageData()) #Set the input data
    mc.ComputeGradientsOn()
    mc.SetValue(0,1) # Binary label map values are from [0,1]
    mc.Update() #Update the marching cubes filters
    return mc.GetOutput() #Return the generated mesh

class OBBTree():
  """ Class to build the OBBTree from the mesh generated by the class MarchingCubes
  this will help us to locate the structure in the space"""
  def build(self, mesh):
    obb = vtk.vtkOBBTree() #Define the method
    obb.SetDataSet(mesh) #Set the input dataset mesh
    obb.BuildLocator() #Build the locator for the mesh
    obb.Update() #Update the tree
    return obb #Return the whole obbtree object

class IntersectWith():
  def run(self, inputVolume, entries, targets):
    """ Class to compute all the none intersecting lines to the critical structure.
    The lines are returned as a vtk.vtkPolyData() type model containing the lines as
    a vtk.vtkCellArray(). The inputVolume is transform into an OBBTree Locator and then
    check if the lines generated by different entries and tragets intersect with the
    mesh or not. To reduce the total number of lines a length threshold was used and
    only the lines shorter than the threshold were saved in order to minimize length"""

    #Set up all the needed functions and parameters for further uses
    lines = vtk.vtkCellArray()
    trajectories = vtk.vtkPolyData()
    points = vtk.vtkPoints()
    mc = MarchingCubes()
    mesh = mc.run(inputVolume)
    obb = OBBTree()
    obbtree = obb.build(mesh)
    #Iterate over the entries
    for i in range(0, entries.GetNumberOfFiducials()):
      entry = [0, 0, 0]
      entries.GetNthFiducialPosition(i, entry) #Get the position for the entry
      #Iterate over the targets that were isolated in PathPlanning from the hippocampus targeting
      for j in range(0, targets.GetNumberOfFiducials()):
        target = [0, 0, 0]
        targets.GetNthFiducialPosition(j, target) #Get the position for the target
        #Use the OBBTree.IntersectWithLine() function to check if the line intersects
        #or not with the mesh. IntersectWithLine recieves two points as inputs one is
        #the starting point of the line and the ending point. Then define what data type
        #correspond to, in this case vtk.vtkPoints().
        intersect = obbtree.IntersectWithLine(entry, target, vtk.vtkPoints(), None)
        #intersect = 0 means the line does not intersect with the structure
        if intersect==0:
          entryId = points.InsertNextPoint(entry[0], entry[1], entry[2]) #Set entry point
          targetId = points.InsertNextPoint(target[0], target[1], target[2]) #Set target point
          line = vtk.vtkLine() #Create the line
          line.GetPointIds().SetId(0, entryId) #Set the 0 point for the line
          line.GetPointIds().SetId(1, targetId) #Set the 1 point for the line
          #Calculate the length of the line
          length = np.sqrt((target[0]-entry[0])**2 + (target[1]-entry[1])**2 + (target[2]-entry[2])**2)
          if length < 55.0: #Check the threshold
            lines.InsertNextCell(line) #Add the line to the CellArray
    #Add the information to the vtk.vtkPolyData() object
    trajectories.SetPoints(points)
    trajectories.SetLines(lines)
    #Add trajectories to the slicer viewer
    pathNode = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLModelNode', 'Trajectories')
    pathNode.SetAndObserveMesh(trajectories)

    return trajectories

class ComputeDistanceImageFromLabelMap():
  def Execute(self, inputVolume):
    sitkInput = su.PullVolumeFromSlicer(inputVolume)
    distanceFilter = sitk.DanielssonDistanceMapImageFilter() # you can try other filters from here: https://itk.org/Doxygen/html/group__ITKDistanceMap.html
    sitkOutput = distanceFilter.Execute(sitkInput)
    outputVolume = su.PushVolumeToSlicer(sitkOutput, None, 'distanceMap')
    return outputVolume

class PathPlanning2Test(ScriptedLoadableModuleTest):
  """
  This is the test case for your scripted module.
  Uses ScriptedLoadableModuleTest base class, available at:
  https://github.com/Slicer/Slicer/blob/master/Base/Python/slicer/ScriptedLoadableModule.py
  """

  def setUp(self):
    """ Do whatever is needed to reset the state - typically a scene clear will be enough.
    """
    slicer.mrmlScene.Clear(0)

  def runTest(self):
    """Run as few or as many tests as needed here.
    """
    self.setUp()
    self.test_LoadData('C:/Users/mikel/Desktop/Healthcare Technologies/Robotic-Software/Practicals/Lab2and3')
    self.test_PathPlanning2_TestNoInterLines()
    self.test_PathPlanning2_TestInterLines()
    self.test_PathPlanning2_TestMarchingCubes()
    self.test_PathPlanning2_TestOBBTree()
    self.test_PathPlanning2_TestNoPoints()
    self.setUp()


  def test_LoadData(self, path):
    self.delayDisplay('Starting load data test')
    isLoaded = slicer.util.loadLabelVolume(path + '/BrainParcellation/ventricles.nii.gz')
    if not isLoaded:
      self.delayDisplay('Unable to load ' + path + '/BrainParcellation/ventricles.nii.gz')

    isLoaded = slicer.util.loadMarkupsFiducialList(path + '/entries.fcsv')
    if not isLoaded:
      self.delayDisplay('Unable to load ' + path + '/entries.fcsv')

    isLoaded = slicer.util.loadMarkupsFiducialList(path + '/targets.fcsv')
    if not isLoaded:
      self.delayDisplay('Unable to load ' + path + '/targets.fcsv')

    self.delayDisplay('Test passed! All data loaded correctly')

  def test_PathPlanning2_TestNoInterLines(self):

    self.delayDisplay('Starting test no intersecting lines')

    mask = slicer.util.getNode('ventricles')

    p1 = slicer.vtkMRMLMarkupsFiducialNode()
    p1.AddFiducial(51, 28, 26)

    p2 = slicer.vtkMRMLMarkupsFiducialNode()
    p2.AddFiducial(53, 17, 28)

    trajectories = IntersectWith().run(mask, p1, p2)

    if trajectories.GetLines().GetSize() == 0:
      self.delayDisplay('Test failed. There is no line')
      return

    self.delayDisplay('Test passed! A line was returned.')

  def test_PathPlanning2_TestInterLines(self):

    self.delayDisplay('Starting test no intersecting lines')

    mask = slicer.util.getNode('ventricles')

    p1 = slicer.vtkMRMLMarkupsFiducialNode()
    p1.AddFiducial(141, 161, 110)

    p2 = slicer.vtkMRMLMarkupsFiducialNode()
    p2.AddFiducial(132, 170, 110)

    trajectories = IntersectWith().run(mask, p1, p2)

    if trajectories.GetLines().GetSize() != 0:
      self.delayDisplay('Test failed. There is an intersecting line')
      return

    self.delayDisplay('Test passed! No line returned.')

  def test_PathPlanning2_TestMarchingCubes(self):

    self.delayDisplay('Starting test for the MarchingCubes')

    mask = slicer.util.getNode('ventricles')
    mesh = MarchingCubes().run(mask)

    if not mesh:
      self.delayDisplay('Test failed. There is no output from MarchingCubes algorithm.')

    self.delayDisplay('Test passed! MarchingCubes algorithm returns an output')

  def test_PathPlanning2_TestOBBTree(self):

    self.delayDisplay('Starting test for the OBBTree')

    mask = slicer.util.getNode('ventricles')
    mesh = MarchingCubes().run(mask)
    obbtree = OBBTree().build(mesh)

    if not obbtree:
      self.delayDisplay('Test failed. There is no output from OBBTree algorithm.')

    self.delayDisplay('Test passed! OBBTree algorithm returns an output')


  def test_PathPlanning2_TestNoPoints(self):
    """Test the case for the null space where there is no entry fiducials"""
    self.delayDisplay('Starting test no points for the ventricles mask')
    ventricles = slicer.util.getNode('ventricles')

    # Empty markup fiducials set
    emptypoints = slicer.vtkMRMLMarkupsFiducialNode()

    # run the class PickPathsmat()
    targets = slicer.util.getNode('targets')
    IntersectWith().run(ventricles, emptypoints, targets)
    self.delayDisplay("Test passed! Empty markup fiducials don't break the class IntersectWith()")