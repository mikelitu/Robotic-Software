import os
import unittest
import numpy as np
import vtk, qt, ctk, slicer, random
import SimpleITK as sitk
from slicer.ScriptedLoadableModule import *
import logging

#
# PathPlanning
#

class PathPlanning(ScriptedLoadableModule):
  """Uses ScriptedLoadableModule base class, available at:
  https://github.com/Slicer/Slicer/blob/master/Base/Python/slicer/ScriptedLoadableModule.py
  """

  def __init__(self, parent):
    ScriptedLoadableModule.__init__(self, parent)
    self.parent.title = "PathPlanning" # TODO make this more human readable by adding spaces
    self.parent.categories = ["Examples"]
    self.parent.dependencies = []
    self.parent.contributors = ["Mikel De Iturrate (King's College London)"] # replace with "Firstname Lastname (Organization)"
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
# PathPlanningWidget
#

class PathPlanningWidget(ScriptedLoadableModuleWidget):
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
    self.inputImageSelector.setMRMLScene( slicer.mrmlScene )
    self.inputImageSelector.setToolTip( "Pick the input to the algorithm." )
    parametersFormLayout.addRow("Input Label Map: ", self.inputImageSelector)


    #
    #input target points
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
    # output target points
    #
    self.outputSelector = slicer.qMRMLNodeComboBox()
    self.outputSelector.nodeTypes = ["vtkMRMLMarkupsFiducialNode"]
    self.outputSelector.selectNodeUponCreation = True
    self.outputSelector.addEnabled = True
    self.outputSelector.removeEnabled = True
    self.outputSelector.noneEnabled = True
    self.outputSelector.showHidden = False
    self.outputSelector.showChildNodeTypes = False
    self.outputSelector.setMRMLScene( slicer.mrmlScene )
    self.outputSelector.setToolTip( "Pick the output fiducials to the algorithm." )
    parametersFormLayout.addRow("Output Paths: ", self.outputSelector)

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
    self.outputSelector.connect("currentNodeChanged(vtkMRMLNode*)", self.onSelect)

    # Add vertical spacer
    self.layout.addStretch(1)

    # Refresh Apply button state
    self.onSelect()

  def cleanup(self):
    pass

  def onSelect(self):
    self.applyButton.enabled = self.inputImageSelector.currentNode() and self.outputSelector.currentNode()

  def onApplyButton(self):
    logic = PathPlanningLogic()
    enableScreenshotsFlag = self.enableScreenshotsFlagCheckBox.checked
    logic.run(self.inputImageSelector.currentNode(), self.inputTargetSelector.currentNode(), self.outputSelector.currentNode(), enableScreenshotsFlag)

#
# PathPlanningLogic
#

class PathPlanningLogic(ScriptedLoadableModuleLogic):
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

  def isValidInputOutputData(self, inputImageNode, inputTargetFiducialsNode, outputFiducialsNode):
    """Validates if the output is not the same as input
    """
    if not inputImageNode:
      logging.debug('isValidInputOutputData failed: no input volume node defined')
      return False
    if not inputTargetFiducialsNode:
      logging.debug('isValidInputOutputData failed: no input fiducial node defined')
      return False
    if not outputFiducialsNode:
      logging.debug('isValidInputOutputData failed: no output fiducial node defined')
      return False
    if inputTargetFiducialsNode.GetID()==outputFiducialsNode.GetID():
      logging.debug('isValidInputOutputData failed: input and output fiducials are the same. Create new fiducials for output to avoid this error.')
      return False
    return True

  def run(self, inputVolume, targets, outtargets, enableScreenshots=0):
    """
    Run the actual algorithm
    """

    if not self.isValidInputOutputData(inputVolume, targets, outtargets):
      slicer.util.errorDisplay('Input volume is the same as output volume. Choose a different output volume.')
      return False
    if not self.hasImageData(inputVolume):
      return False

    logging.info('Processing started')

    # Compute the path selection algorithm
    pathPicker = PickPathsmat()
    pathPicker.run(inputVolume, targets, outtargets)

    # Capture screenshot
    if enableScreenshots:
      self.takeScreenshot('PathPlanningTest-Start','MyScreenshot',-1)

    logging.info('Processing completed')

    return True


class PickPathsmat():
  def run(self, inputVolume, targets, outtargets):
    #Improve method for the computation of the first task of the path planning assignment, in this case instead of using
    #the points in the global world coordinate, the points are translated to the image coordinates and then the pixel value
    #is computed using the same process as before.
    outtargets.RemoveAllMarkups()
    #Define the matrix with the information of the image axis orientation
    ref = vtk.vtkMatrix4x4()
    inputVolume.GetRASToIJKMatrix(ref)
    #Set the matrix as a vtk transformation matrix
    trans = vtk.vtkTransform()
    trans.SetMatrix(ref)
    for x in range(0, targets.GetNumberOfFiducials()):
      target = [0, 0, 0]
      targets.GetNthFiducialPosition(x, target)
      #Transform the point
      ind = trans.TransformPoint(target)
      #Compute the value of the pixel at that point, remember that the indexes must be integers to index in the input volume
      pixelValue = inputVolume.GetImageData().GetScalarComponentAsDouble(int(ind[0]), int(ind[1]), int(ind[2]), 0)
      if pixelValue == 1:
          outtargets.AddFiducial(target[0], target[1], target[2])



class PathPlanningTest(ScriptedLoadableModuleTest):
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
    self.test_PathPlanning_OutFiducial()
    self.test_PathPlanning_InFiducial()
    self.test_PathPlanning_TestEmptyMask()
    self.test_PathPlanning_TestNoPoints()
    self.setUp() #clear all the data after testing

  def test_LoadData(self, path):
    """ Ideally you should have several levels of tests.  At the lowest level
    tests should exercise the functionality of the logic with different inputs
    (both valid and invalid).  At higher levels your tests should emulate the
    way the user would interact with your code and confirm that it still works
    the way you intended.
    One of the most important features of the tests is that it should alert other
    developers when their changes will have an impact on the behavior of your
    module.  For example, if a developer removes a feature that you depend on,
    your test should break so they know that the feature is needed.
    """

    self.delayDisplay("Starting the load data test")
    #Load some data for the rest of the tests to work. This test checks that the
    #desire data files are at the expected directory

    isLoaded = slicer.util.loadLabelVolume(path + '/BrainParcellation/r_hippo.nii.gz')
    if not isLoaded:
      self.delayDisplay('Unable to load ' + path + '/BrainParcellation/r_hippo.nii.gz')

    isLoaded = slicer.util.loadMarkupsFiducialList(path + '/targets.fcsv')
    if not isLoaded:
      self.delayDisplay('Unable to load ' + path + '/targets.fcsv')

    self.delayDisplay('Test passed! All data loaded correctly')

  def test_PathPlanning_OutFiducial(self):
    #Check that the algorithm doesn't save a fiducial that's out of the target
    self.delayDisplay('Starting test point is outside target.')

    #get image node
    hippo = slicer.util.getNode('r_hippo')

    #Code two point that is outside the target structure
    PointOut = slicer.vtkMRMLMarkupsFiducialNode()
    PointOut.AddFiducial(228, 94, 110) #This point was obtained by inspection
    PointOut.AddFiducial(0, 0, 0) #Point obtained by inspection

    #run the class PickPathsmat
    Output = slicer.vtkMRMLMarkupsFiducialNode()
    PickPathsmat().run(hippo, PointOut, Output)

    #check if it has returned any fiducial -- it has to be empty
    if Output.GetNumberOfFiducials() > 0:
      self.delayDisplay('Test failed. There are ' + str(Output.GetNumberOfFiducials()))
      return

    self.delayDisplay('Test passed! No points were returned')

  def test_PathPlanning_InFiducial(self):
    #Check that the algorithm doesn't save a fiducial that's out of the target
    self.delayDisplay('Starting test point is inside target.')

    #get image node
    hippo = slicer.util.getNode('r_hippo')

    #Code two points that is inside the target structure
    PointIn = slicer.vtkMRMLMarkupsFiducialNode()
    PointIn.AddFiducial(156, 122, 110) #This point was obtained by inspection
    PointIn.AddFiducial(153.6, 126.1, 103) #This point was obtained by inspection

    #run the class PickPathsmat
    Output = slicer.vtkMRMLMarkupsFiducialNode()
    PickPathsmat().run(hippo, PointIn, Output)

    #check if it has returned any fiducial -- it has to be empty
    if Output.GetNumberOfFiducials() != 2:
      self.delayDisplay('Test failed. There are ' + str(Output.GetNumberOfFiducials()))
      return

    self.delayDisplay('Test passed! ' + str(Output.GetNumberOfFiducials()) + ' points were returned')

  def test_PathPlanning_TestEmptyMask(self):
    """Test the case for the empty mask where there is no target mask"""
    self.delayDisplay('Starting test points for empty mask')
    emptymask = slicer.vtkMRMLLabelMapVolumeNode()
    emptymask.SetAndObserveImageData(vtk.vtkImageData())

    targets = slicer.util.getNode('targets')

    #run the class PickPathsmat()
    Output = slicer.vtkMRMLMarkupsFiducialNode()
    PickPathsmat().run(emptymask, targets, Output)
    self.delayDisplay("Test passed! Empty mask doesn't break the class PickPathsmat()")

  def test_PathPlanning_TestNoPoints(self):
    """Test the case for the null space where there is no target mask"""
    self.delayDisplay('Starting test no points for the hippocampus mask')
    hippo = slicer.util.getNode('r_hippo')

    #Empty markup fiducials set
    emptypoints = slicer.vtkMRMLMarkupsFiducialNode()

    #run the class PickPathsmat()
    Output = slicer.vtkMRMLMarkupsFiducialNode()
    PickPathsmat().run(hippo, emptypoints, Output)
    self.delayDisplay("Test passed! Empty markup fiducials don't break the class PickPathsmat()")

