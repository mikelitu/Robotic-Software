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
    #input entry points
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
    # output paths
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
    logic.run(self.inputImageSelector.currentNode(), self.inputEntrySelector.currentNode(), self.inputTargetSelector.currentNode(), self.outputSelector.currentNode(), enableScreenshotsFlag)

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

  def isValidInputOutputData(self, inputVolumeNode, outputFiducialsNode):
    """Validates if the output is not the same as input
    """
    if not inputVolumeNode:
      logging.debug('isValidInputOutputData failed: no input volume node defined')
      return False
    if not outputFiducialsNode:
      logging.debug('isValidInputOutputData failed: no output volume node defined')
      return False
    if inputVolumeNode.GetID()==outputFiducialsNode.GetID():
      logging.debug('isValidInputOutputData failed: input and output volume is the same. Create a new volume for output to avoid this error.')
      return False
    return True

  def run(self, inputVolume, entries, targets, outpaths, enableScreenshots=0):
    """
    Run the actual algorithm
    """

    if not self.isValidInputOutputData(inputVolume, outpaths):
      slicer.util.errorDisplay('Input volume is the same as output volume. Choose a different output volume.')
      return False

    logging.info('Processing started')

    # Compute the thresholded output volume using the Threshold Scalar Volume CLI module
    pathPicker = PickPaths()
    pathPicker.run(inputVolume, entries, targets, outpaths)

    # Capture screenshot
    if enableScreenshots:
      self.takeScreenshot('PathPlanningTest-Start','MyScreenshot',-1)

    logging.info('Processing completed')

    return True

class Line():
  def getPoints(self, point_a, point_b, scaling = 0.1):
    points = []
    points.append(point_a)
    diff = list(np.array(point_b) - np.array(point_a))
    new_point = point_a
    while new_point != point_b:
      new_point = new_point + list(scaling * np.array(diff))
      points.append(new_point)
    return points


class PickPaths():
  def run(self, inputVolume, entries, targets, outpaths):
    #Given an input and a single point the algorithm will return the value
    #of the pixel at that certain position
    outpaths.RemoveAllMarkups()
    out_points = []
    for y in range(0, entries.GetNumberOfFiducials()):
      entry = [0, 0, 0]
      entries.GetNthFiducialPosition(y, entry)
      for x in range(0, targets.GetNumberOfFiducials()):
        target = [0, 0, 0]
        targets.GetNthFiducialPosition(x, target)
        myline = Line()
        points = myline.getPoints(entry, target)
        for point in points:
          ind = inputVolume.GetImageData().FindPoint(point)
          dim = inputVolume.GetImageData().GetDimensions()
          xind = ind % dim[0]
          yind = (ind % (dim[0] * dim[1])) / dim[0]
          zind = ind / (dim[0] * dim[1])
          pixelValue = inputVolume.GetImageData().GetScalarComponentAsDouble(xind, yind, zind, 0)
          if pixelValue == 1 & point not in out_points:
            outpaths.AddFiducial(point[0], point[1], point[2])
            out_points.append(point)

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
    self.test_PathPlanning1()

  def test_PathPlanning1(self):
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

    self.delayDisplay("Starting the test")
    #
    # first, get some data
    #
    import SampleData
    SampleData.downloadFromURL(
      nodeNames='FA',
      fileNames='FA.nrrd',
      uris='http://slicer.kitware.com/midas3/download?items=5767')
    self.delayDisplay('Finished with download and loading')

    volumeNode = slicer.util.getNode(pattern="FA")
    logic = PathPlanningLogic()
    self.assertIsNotNone( logic.hasImageData(volumeNode) )
    self.delayDisplay('Test passed!')
