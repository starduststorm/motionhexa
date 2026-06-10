#!/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3
# -*- coding: utf-8 -*-
import os
import sys
from pathlib import Path
from math import *
sys.path.insert(0, str(Path(__file__).parent.parent / "lib" / "dustlib" / "pcbtools"))
 
from pcblayout import *
import pcbnew_compat as pcb

def is_point_in_triangle(P, A, B, C):
    # barycenter method
    def area(AA, BB, CC):
        return abs((AA.x * (BB.y - CC.y) + BB.x * (CC.y - AA.y) + CC.x * (AA.y - BB.y)) / 2.0)
    A_total = area(A, B, C)
    A1 = area(P, B, C)
    A2 = area(A, P, C)
    A3 = area(A, B, P)
    return abs(A_total - A1 - A2 - A3) < 0.0001

class LayoutHexa(PCBLayout):
  edgeRadius = 39
  pixelRadius = 38
  pixelSpacing = 3.9

  ##
  
  _hexaPoints = {}
  def hexaPoints(self, radius):
    if radius not in self._hexaPoints:
      self._hexaPoints[radius] = []
      for i in range(6):
        x = radius * cos(i * 2*pi / 6)
        y = radius * sin(i * 2*pi / 6)
        self._hexaPoints[radius].append(Point(x,y))
    return self._hexaPoints[radius]

  def drawEdgeCuts(self):
    super().drawEdgeCuts()

    points = self.hexaPoints(self.edgeRadius)
    lastPoint = points[-1]
    for pos in points:
      self.drawSegment(lastPoint, pos, 'Edge.Cuts', self.edge_cut_line_thickness)
      lastPoint = pos
    
    zonePoints = self.hexaPoints(self.edgeRadius+2)
    for pos in zonePoints:
      self.addZonePoint(pos, "GND1", "GND", "In2.Cu")
      self.addZonePoint(pos, "+5V", "+5V", "In1.Cu")
      

  def is_point_in_hexa(self, pos, radius=None):
    # decide if contained in the hexa, by measuring if the point is in any of the component triangles
    radius = self.pixelRadius if radius is None else radius
    points = self.hexaPoints(radius)
    for i in range(6):
      p1 = Point(0,0)
      p2 = points[i]
      p3 = points[(i+1)%len(points)]
      if is_point_in_triangle(pos, p1,p2,p3):
        print("Point {} in triangle {} {} {}".format(pos, p1, p2, p3))
        return True
    return False

  def placeZigZag(self):
    spacing = Point(self.pixelSpacing, sin(2*pi/6) * self.pixelSpacing)
    # i wanna have a pixel at 0,0 but still have row-major zig-zag wiring order for layout/schematic sanity reasons
    def findTopRightPlacement():
      pos = Point(0,0)
      while self.is_point_in_hexa(pos):
        pos += (spacing.x, 0)
      rightmost = pos.x
      pos = Point(0,0)
      while self.is_point_in_hexa(pos):
        pos -= (0, spacing.y)
      topmost = pos.y
      return Point(rightmost, topmost)

    TraceBuilder.defaultTraceWith = 0.15

    startPos = findTopRightPlacement()
    print("startPos = ", startPos);
    pos = startPos - (self.pixelSpacing, 0)
    xDirection = -1
    placedLeftCap = False
    placedRightCap = False
    capIndex = 1
    capPrototype = pcb.FootprintLoad(str(Path(findFootprintLibsFolder()).joinpath("Capacitor_SMD.pretty")), "C_0402_1005Metric")

    def insertCap(pos, orientation):
      nonlocal capIndex
      reference = "C{}".format(capIndex)
      cap = self.insertFootprint(capPrototype, reference, pos, orientation, "F.Cu")
      capIndex+=1
      for i, pad in enumerate(cap.Pads()):
        TraceBuilder(pad).angleConst(0.4, -orientation + i*pi - pi).via().draw(self.kicadpcb)
      return cap

    while True:
      orientation = -pi/4
      placed = None
      if self.is_point_in_hexa(pos):
        if pos.x < 0 and xDirection > 0 and not placedLeftCap:
          angle = -3*pi/4
          angleAdjust = 0 if pos.y < 0 else -pi/2
          cap = insertCap(pos.polar_translated(2.4,angle+angleAdjust), orientation+angleAdjust+pi/2)
          placedLeftCap = True

        def footprintPadNamed(fp, padName):
          for pad in fp.Pads():
            if pad.GetPadName() == padName:
              return pad
          return None

        def dataTraceConnector(fromPad, toPad):
          start = Point(fromPad.GetPosition())
          end = Point(toPad.GetPosition())
          six_to_four = fromPad.GetPadName() == "6"
          one_to_three = fromPad.GetPadName() == "1"

          if xDirection > 0:
            if end.y - start.y > 2:
              # new row, left side of hexa
              # left side leaves pad 6 the same
              if six_to_four:
                  tb1 = TraceBuilder(fromPad).angleConst(0.5, pi/4).via().angleConst(0.84, pi)
              if start.y < self.center.y-1.5:
                # new row, top left
                if six_to_four:
                  pivot = TraceBuilder(footprintPadNamed(fromPad.GetParentFootprint(), "4")).xconst(-1.0).useAsPivot()
                  tb1.joinWithCardialAtSplitRatio(0.5, pivot)
                  tb2 = TraceBuilder(toPad).angleConst(0.25, 3*pi/4).via().yconst(-0.8)
                  tb1.joinWithCardialAtSplitRatio(0.5, tb2).draw(self.kicadpcb)
                if one_to_three:
                  pivot = TraceBuilder(footprintPadNamed(fromPad.GetParentFootprint(), "3")).xconst(-0.85).useAsPivot()
                  tb1 = TraceBuilder(fromPad).angleConst(0.4, pi/4).via()
                  tb1.joinWithCardialAtSplitRatio(0.49, pivot)
                  tb2 = TraceBuilder(toPad).angleConst(0.5, pi/4).via()
                  tb1.joinWithCardialAtSplitRatio(0.28, tb2).draw(self.kicadpcb)
                
              else:
                # new row, bottom left
                if six_to_four:
                  pivot = TraceBuilder(footprintPadNamed(fromPad.GetParentFootprint(), "4")).xconst(-0.8).useAsPivot()
                  tb1.joinWithCardialAtSplitRatio(0.7, pivot)
                  tb2 = TraceBuilder(toPad).angleConst(0.5, -3*pi/4).via()
                  tb2.octPathCloseTo(tb1).draw(self.kicadpcb)
                if one_to_three:
                  # use pad 3 of the start pixel as a pivot
                  pivot = TraceBuilder(footprintPadNamed(fromPad.GetParentFootprint(), "3")).xconst(-0.85).useAsPivot()
                  tb1 = TraceBuilder(fromPad).angleConst(0.16, 3*pi/4).via().yconst(0.25)
                  tb1.joinWithCardialAtSplitRatio(0.5, pivot).yconst(0.8)
                  tb2 = TraceBuilder(toPad).angleConst(0.6, 3*pi/4).via()
                  tb1.joinWithCardialAtSplitRatio(0.3, tb2).draw(self.kicadpcb)
            else:
              # regular rightward row
              if six_to_four:
                tb = TraceBuilder(fromPad).xconst(xDirection*0.4)
              if one_to_three:
                tb = TraceBuilder(fromPad).angleConst(0.4, pi/4).xconst(1.0)
              tb.octPathCloseTo(toPad).draw(self.kicadpcb)
          
          else: # xDirection >= 0
            # leftward, always land on the pad the same way
            if six_to_four:
              tb2 = TraceBuilder(toPad).angleConst(0.5, pi/2).via()
            if one_to_three:
              tb2 = TraceBuilder(toPad).angleConst(0.3, 3*pi/4).via()
            
            if end.y - start.y > 2:
              # new row, right side of hexa
              if start.y < self.center.y:
                # new row, top right
                if six_to_four:
                  tb1 = TraceBuilder(fromPad).angleConst(0.8, 0).via()
                  tb2 = tb2.angleConst(1.14, -pi/4).xconst(1.1122).angleConst(0.41, -pi/4).yconst(-1.9)
                  tb1.octPathCloseTo(tb2).draw(self.kicadpcb)
                if one_to_three:
                  tb1 = TraceBuilder(fromPad).angleConst(0.5, pi/4).via()
                  tb2 = tb2.xconst(-xDirection*1.5).angleConst(1.25, -pi/4)
                  tb1.octPathCloseTo(tb2).draw(self.kicadpcb)
                  
              else:
                # new row, bottom right
                if six_to_four:
                  tb1 = TraceBuilder(fromPad).angleConst(0.16, 3*pi/4).via().angleConst(0.7, 3*pi/4).yconst(1.4)
                  tb2 = tb2.angleConst(1.14, -pi/4).xconst(1.1160)
                  tb1.joinWithCardialAtSplitRatio(0.57, tb2).draw(self.kicadpcb)
                if one_to_three:
                  tb1 = TraceBuilder(fromPad).angleConst(0.5, pi/4).via().angleConst(0.5, 3*pi/4)
                  tb2 = tb2.xconst(1.45)
                  tb2.octPathCloseTo(tb1).draw(self.kicadpcb)
            else:
              # regular leftward row
              srcPadDistance = 0.5 if six_to_four else 0.4
              dataTraceSplit = 0.427 if six_to_four else 0.46
              if six_to_four:
                tb1 = TraceBuilder(fromPad).angleConst(srcPadDistance, pi/4).via().xconst(-0.77)
              else:
                tb1 = TraceBuilder(fromPad).angleConst(srcPadDistance, pi/4).via()
                tb2 = tb2.xconst(-xDirection*1.32)
              combined = tb1.joinWithCardialAtSplitRatio(dataTraceSplit, tb2).draw(self.kicadpcb)

        gndDistance = 0.5 if xDirection > 0 else 0.65
        # connect power via slightly different on the rightward to avoid caps
        powerConnect = TraceBuilder().angleConst(0.7256, 0).angleConst(0.2387, -pi/4).via() if xDirection>0 else TraceBuilder().angleConst(0.8, 0).angleConst(0.35, pi/4).via()
        placed = self.placeSeriesPixel(pos, orientation, allowOverlaps=False, alignOverlaps=False, 
                                       powerConnect=powerConnect,
                                       groundConnect=TraceBuilder().angleConst(gndDistance, -pi).via(),
                                       connectorFunction=dataTraceConnector)

        # draw lil hexa
        for i in range(6):
          pt1 = pos.polar_translated(self.pixelSpacing/2, i*2*pi/6)
          pt2 = pos.polar_translated(self.pixelSpacing/2, (i+1)*2*pi/6)
          self.drawSegment(pt1, pt2, "F.Silkscreen", 0.15)
          
      else:
        if pos.x > 0 and xDirection > 0 and not placedRightCap:
          # just fell off the right end of the hexa
          lastPos = pos - (xDirection*spacing.x, 0)
          angle = -pi/4
          angleAdjust = 0 if lastPos.y < 0 else pi/2
          insertCap(lastPos.polar_translated(2.4,angle+angleAdjust), orientation+angleAdjust)
          placedRightCap = True

      if pos.x < -startPos.x or pos.x > startPos.x: # centered around 0 so we can do this
        # next row
        assert(not placed)
        pos += (-xDirection * 3*spacing.x/2, spacing.y)
        if pos.y > -startPos.y:
          break
        xDirection *= -1
        placedLeftCap = False
        placedRightCap = False
      else:
        pos += (xDirection * spacing.x, 0)

  def placePixels(self):
    super().placePixels()
    self.placeZigZag()

  def decorateSilkScreen(self):
    super().decorateSilkScreen()

args = getArgs()
footprintsPath = str(Path(__file__).parent.parent / 'kicad_footprints.pretty')
layout = LayoutHexa(args.path, SK9822EC20, footprintsPath, args=args)

if __name__ == '__main__':
  sys.exit(0 if layout.doLayout() else 1)

