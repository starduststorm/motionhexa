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

  # Row-major tuning parameters (mm)
  capRowStride = 2      # place caps every N rows

  def enumerateRows(self):
    spacing = Point(self.pixelSpacing, sin(2*pi/6) * self.pixelSpacing)
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

    startPos = findTopRightPlacement()
    pos = startPos - (self.pixelSpacing, 0)
    xDirection = -1
    rows = []
    currentRow = []
    while True:
      if self.is_point_in_hexa(pos):
        currentRow.append(Point(pos.x, pos.y))
      if pos.x < -startPos.x or pos.x > startPos.x:
        if currentRow:
          rows.append(sorted(currentRow, key=lambda p: p.x))
          currentRow = []
        pos += (-xDirection * 3*spacing.x/2, spacing.y)
        if pos.y > -startPos.y:
          break
        xDirection *= -1
      else:
        pos += (xDirection * spacing.x, 0)
    if currentRow:
      rows.append(sorted(currentRow, key=lambda p: p.x))
    return rows

  def placeRowMajor(self):
    TraceBuilder.defaultTraceWith = 0.15
    orientation = -pi/4

    capIndex = 1
    capPrototype = pcb.FootprintLoad(str(Path(findFootprintLibsFolder()).joinpath("Capacitor_SMD.pretty")), "C_0402_1005Metric")

    def insertCap(pos, capOrientation):
      nonlocal capIndex
      reference = "C{}".format(capIndex)
      cap = self.insertFootprint(capPrototype, reference, pos, capOrientation, "F.Cu")
      capIndex += 1
      for i, pad in enumerate(cap.Pads()):
        TraceBuilder(pad).angleConst(0.4, -capOrientation + i*pi - pi).via().draw(self.kicadpcb)
      return cap

    rows = self.enumerateRows()
    print("enumerateRows: {} rows, {} total pixels".format(len(rows), sum(len(r) for r in rows)))

    for rowIndex, row in enumerate(rows):
      placeCaps = (rowIndex % self.capRowStride == 0)
      if placeCaps and row:
        firstPos = row[0]
        adj = 0 if firstPos.y < 0 else -pi/2
        insertCap(firstPos.polar_translated(2.4, -3*pi/4 + adj), orientation + adj + pi/2)

      for pos in row:
        self.placeSeriesPixel(pos, orientation, allowOverlaps=False, alignOverlaps=False,
          powerConnect=TraceBuilder().angleConst(0.7256, 0).angleConst(0.2387, -pi/4).via(),
          groundConnect=TraceBuilder().angleConst(0.5, -pi).via(),
          connectorFunction=self.rowMajorConnector)
        # draw lil hexa silkscreen marker
        for i in range(6):
          pt1 = pos.polar_translated(self.pixelSpacing/2, i*2*pi/6)
          pt2 = pos.polar_translated(self.pixelSpacing/2, (i+1)*2*pi/6)
          self.drawSegment(pt1, pt2, "F.Silkscreen", 0.15)

      if placeCaps and row:
        lastPos = row[-1]
        adj = 0 if lastPos.y < 0 else pi/2
        insertCap(lastPos.polar_translated(2.4, -pi/4 + adj), orientation + adj)

  def rowMajorConnector(self, fromPad, toPad):
    six_to_four = fromPad.GetPadName() == "6"
    one_to_three = fromPad.GetPadName() == "1"
    srcC = Point(fromPad.GetParentFootprint().GetPosition()) - self.center
    dstC = Point(toPad.GetParentFootprint().GetPosition()) - self.center
    rowTransition = dstC.y - srcC.y > 2

    if not rowTransition:
      # same-row rightward traces on F.Cu
      if six_to_four:
        TraceBuilder(fromPad).xconst(0.4).octPathCloseTo(toPad).draw(self.kicadpcb)
      if one_to_three:
        TraceBuilder(fromPad).angleConst(0.4, pi/4).xconst(1.0).octPathCloseTo(toPad).draw(self.kicadpcb)
      return

    # row transition: bridge through In1.Cu underneath the same row, over to start of next row
    if one_to_three:
      toBuilder = TraceBuilder(toPad).angleConst(0.2, 3*pi/4).via(toLayer="In1.Cu").xconst(0.64)
      (TraceBuilder(fromPad)
        .angleConst(0.5, pi/4).via(toLayer="In1.Cu")
        .joinWithCardinalAtSplitRatio(0.15, toBuilder)
        .draw(self.kicadpcb))

    if six_to_four:
      toBuilder = TraceBuilder(toPad).angleConst(-0.95, 0).via(toLayer="In1.Cu").yconst(-0.0973)
      (TraceBuilder(fromPad)
        .angleConst(-0.3, pi).via(toLayer="In1.Cu")
        .joinWithCardinalAtSplitRatio(0.07, toBuilder)
        .draw(self.kicadpcb))

  def checkIn1Clearances(self):
    board = self.kicadpcb.board
    in1_id = self.kicadpcb.layertable.get("In1.Cu")
    vias = []
    in1_segs = []
    for t in board.GetTracks():
      if type(t) is pcb.PCB_VIA:
        vias.append(Point(t.GetPosition()) - self.center)
      elif type(t) is pcb.PCB_TRACK and t.GetLayer() == in1_id:
        in1_segs.append((Point(t.GetStart()) - self.center, Point(t.GetEnd()) - self.center))
    min_vv = 0.76
    min_sv = 0.555
    issues = 0
    for i, v1 in enumerate(vias):
      for v2 in vias[i+1:]:
        d = v1.distance_to(v2)
        if d < min_vv:
          print("VIA-VIA CLASH {:.3f}mm: {} and {}".format(d, v1, v2))
          issues += 1
    for (s, e) in in1_segs:
      for v in vias:
        # point-to-segment distance
        seg = e - s
        seg_len = s.distance_to(e)
        if seg_len < 0.0001:
          d = s.distance_to(v)
        else:
          t = max(0, min(1, ((v.x-s.x)*seg.x + (v.y-s.y)*seg.y) / (seg_len**2)))
          proj = s + Point(seg.x*t, seg.y*t)
          d = proj.distance_to(v)
        if d < min_sv and d > 0.05:  # skip via endpoints themselves
          print("SEG-VIA CLASH {:.3f}mm on In1.Cu: seg ({})→({}) vs via {}".format(d, s, e, v))
          issues += 1
    print("checkIn1Clearances: {} issues found".format(issues))

  def placePixels(self):
    super().placePixels()
    self.placeRowMajor()

  def decorateSilkScreen(self):
    super().decorateSilkScreen()

args = getArgs()
footprintsPath = str(Path(__file__).parent.parent / 'kicad_footprints.pretty')
layout = LayoutHexa(args.path, SK9822EC20, footprintsPath, args=args)

if __name__ == '__main__':
  sys.exit(0 if layout.doLayout() else 1)

