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
    self.enableScreenshotsFlagCheckBox.setToolTip(
      "If checked, take screen shots for tutorials. Use Save Data to write them to disk.")
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
    self.applyButton.enabled = self.inputImageSelector.currentNode()

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

  def isValidInputOutputData(self, inputVolumeNode):
    """Validates if the output is not the same as input
    """
    if not inputVolumeNode:
      logging.debug('isValidInputOutputData failed: no input volume node defined')
      return False
    return True

  def run(self, inputVolume, entries, targets, enableScreenshots=0):
    """
    Run the actual algorithm
    """

    logging.info('Processing started')

    # Compute the thresholded output volume using the Threshold Scalar Volume CLI module
    intersect = IntersectWith()
    intersect.run(inputVolume, entries, targets)

    # Capture screenshot
    if enableScreenshots:
      self.takeScreenshot('PathPlanning2Test-Start','MyScreenshot',-1)

    logging.info('Processing completed')

    return True

class MarchingCubes():
  def run(self, inputImage):
    mc = vtk.vtkMarchingCubes()
    mc.SetInputData(inputImage.GetImageData())
    mc.ComputeGradientsOn()
    mc.SetValue(0,1)
    mc.Update()
    return mc.GetOutput()

class OBBTree():
  def build(self, mesh):
    obb = vtk.vtkOBBTree()
    obb.SetDataSet(mesh)
    obb.BuildLocator()
    obb.Update()
    return obb

class IntersectWith():
  def run(self, inputVolume, entries, targets):
    lines = vtk.vtkCellArray()
    trajectories = vtk.vtkPolyData()
    points = vtk.vtkPoints()
    mc = MarchingCubes()
    mesh = mc.run(inputVolume)
    obb = OBBTree()
    obbtree = obb.build(mesh)
    for i in range(0, entries.GetNumberOfFiducials()):
      entry = [0, 0, 0]
      entries.GetNthFiducialPosition(i, entry)
      for j in range(0, targets.GetNumberOfFiducials()):
        target = [0, 0, 0]
        targets.GetNthFiducialPosition(j, target)
        intersect = obbtree.IntersectWithLine(entry, target, vtk.vtkPoints(), None)
        if intersect==0:
          entryId = points.InsertNextPoint(entry[0], entry[1], entry[2])
          targetId = points.InsertNextPoint(target[0], target[1], target[2])
          line = vtk.vtkLine()
          line.GetPointIds().SetId(0, entryId)
          line.GetPointIds().SetId(1, targetId)
          length = np.sqrt((target[0]-entry[0])**2 + (target[1]-entry[1])**2 + (target[2]-entry[2])**2)
          if length < 55.0:
            lines.InsertNextCell(line)
    trajectories.SetPoints(points)
    trajectories.SetLines(lines)
    pathNode = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLModelNode', 'Trajectories')
    pathNode.SetAndObserveMesh(trajectories)


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
    self.test_PathPlanning21()

  def test_PathPlanning21(self):
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

    def setUp(self):
      slicer.mrmlScene.Clear(0)

    def runTest(self):
      self.setUp()
      self.test_LoadData('C:/Users/mikel/Desktop/Healthcare Technologies/Robotic-Software/Practicals/Lab2and3/TestSet')
      self.test_PathPlanning2_NoInterLines()
      #self.test_PathPlanning2_InterLines()
      self.setUp()

    def test_LoadData(self, path):
      self.delayDisplay('Starting load data test')
      isLoaded = slicer.util.loadLabelVolume(path + 'ventriclesTest.nii.gz')
      if not isLoaded:
        self.delayDisplay('Unable to load' + path + 'ventriclesTest.nii.gz')

      self.delayDisplay('Test passed! All data loaded correctly')

    def test_PathPlanning2_NoInterLines():

      self.delayDisplay('Starting test no intersecting lines')

      mask = slicer.util.getNode('ventriclesTest')

      p1 = slicer.vtkMRMLMarkupsFiducialNode()
      p1.AddFiducial(51, 28, 26)

      p2 = slicer.vtkMRMLMarkupsFiducialNode()
      p2.AddFiducial(53, 17, 28)

      trajectories = IntersectWith().run(mask, p1, p2)

      if trajectories.GetLines().GetSize() == 0:
        self.delayDisplay('Test failed. There is no line')
        return

      self.delayDisplay('Test passed! A line was returned.')

