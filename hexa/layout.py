#!/Applications/Kicad/kicad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3
# -*- coding: utf-8 -*-
import os
import sys
from pathlib import Path
from math import *
import cmath
import argparse
from urllib.request import getproxies

parser = argparse.ArgumentParser()
parser.add_argument('-p', "--path", action='store', required=True, help="Path to kicad_pcb file")
parser.add_argument('-L', "--do-layout", action='store_true', required=False, help="Run main layout")
parser.add_argument("--do-silk", action='store_true', required=False, help="Draw silk if not doing main layout")
parser.add_argument('--dry-run', action='store_true', help='Don\'t save results')

# drawing
parser.add_argument('--draw-poly', action='store', help='Draw a polygon with format "sides,radius,layer"')
parser.add_argument('--draw-wave', action='store', help='Draw a wave with format "startx,starty,endx,endy,cycles,amplitude,segments,layer"')
parser.add_argument('--draw-ellipse', action='store', help='Draw an ellipse with format "centerx,centery,xradius,yradius,rotation,segments,layer"')
parser.add_argument('--zone-circle', nargs=6, type=str, help='Draw a zone with the given x, y, radius, name, net, and layer')

# editing
parser.add_argument('--delete-all-traces', action='store_true', help='Delete All Traces in the pcbnew and then exit')
parser.add_argument('--delete-all-drawings', action='store_true', help='Delete all lines & texts in the pcbnew and then exit')
parser.add_argument('--hide-pixel-labels', action='store_true', help="For all footprints like D*, set reference text to hidden")
parser.add_argument('--hide-all-labels', action='store_true', help="Set reference text hidden for all footprints")

# options
parser.add_argument('--skip-traces', action='store_true', help='Don\'t add traces')
parser.add_argument('--lock', action='store_true', default=False, help='Lock pcb elements')

# info
parser.add_argument('--dump-objects', action='store', nargs='?', const=True, default=None)
parser.add_argument('--stats', action='store_true', default=False)


args = parser.parse_args()

# sys.path.insert(0, "/Applications/Kicad/kicad.app/Contents/Frameworks/python/site-packages/")
# sys.path.insert(0, "/Applications/Kicad/kicad.app/Contents/Frameworks/Python.framework/Versions/Current/lib/python3.9/site-packages")
sys.path.append("/usr/local/lib/python3.9/site-packages")

import pcbnew
IU_PER_MM = pcbnew.PCB_IU_PER_MM

_footprintsLibFolder = None
def findFootprintLibsFolder():
  # No way I can find to get KICAD7_FOOTPRINT_DIR from outside the app
  # on my system it's /Applications/KiCad/KiCad.app/Contents/SharedSupport/footprints
  global _footprintsLibFolder
  if _footprintsLibFolder is None:
    path = Path(pcbnew.__file__).parent
    while "SharedSupport" not in (p.stem for p in path.iterdir()):
      path = path.parent
      assert path != path.parent, "failed to find footprints lib"
    _footprintsLibFolder = path.joinpath("SharedSupport/footprints")
  return _footprintsLibFolder

class BoardConstraints(object):
  minDrill = 0
  annularRing = 0
  def __init__(self, minDrill, annularRing):
    self.minDrill = minDrill
    self.annularRing = annularRing

JLPCBRigid2Layer = BoardConstraints(minDrill=0.3, annularRing=0.13)

BoardConstraints = JLPCBRigid2Layer

def sign(x):
  return 1 if x >= 0 else -1

def circle_pt(theta, radius):
  return Point(radius*cos(theta), radius*sin(theta))

def mkfloat(*args):
  return (float(a) for a in args)

class Point(object):
  @classmethod
  def fromWxPoint(cls, wxPoint):
    return Point(wxPoint.x / IU_PER_MM, wxPoint.y / IU_PER_MM)

  @classmethod
  def fromVector2i(cls, vector):
    return Point(vector.x / IU_PER_MM, vector.y / IU_PER_MM)

  @classmethod
  def fromComplex(cls, c):
    return Point(c.real, c.imag)

  def __init__(self, arg1, arg2=None):
    if arg2 is not None:
      self.x = arg1
      self.y = arg2
    elif type(arg1) == pcbnew.wxPoint:
      p = Point.fromWxPoint(arg1)
      self.x = p.x
      self.y = p.y
    elif type(arg1) == pcbnew.VECTOR2I:
      p = Point.fromVector2i(arg1)
      self.x = p.x
      self.y = p.y
    elif type(arg1) == complex:
      p = Point.fromComplex(arg1)
      self.x = p.x
      self.y = p.y
    else:
      self.x = arg1[0]
      self.y = arg1[1]

  def wxPoint(self):
    return pcbnew.wxPoint(self.x * IU_PER_MM , self.y * IU_PER_MM)
  
  def vector2i(self):
    return pcbnew.VECTOR2I_MM(self.x, self.y)

  def translate(self, vec):
    self.x += vec.x
    self.y += vec.y

  def translated(self, vec):
    return Point(self.x+vec.x, self.y+vec.y)

  def polar_translated(self, distance, angle):
    vec = distance * cmath.exp(angle * 1j)
    return self.translated(Point.fromComplex(vec))

  def rotate_about(self, theta, center):
    self.translate(-center)
    self.x, self.y = self.x*cos(theta) - self.y*sin(theta), self.x*sin(theta) + self.y*cos(theta)
    self.translate(center)
    return self

  def __getattr__(self, attr):
    if attr == "theta":
      if self.x == 0:
        return pi/2 if self.y > 0 else 3*pi/2
      theta = atan(self.y/self.x)
      if self.x > 0 and self.y > 0:
        return theta
      elif self.x > 0 and self.y < 0:
        return 2*pi + theta
      else:
        return pi + theta
    elif attr == "radius":
      return sqrt(self.x**2 + self.y**2)
    else:
      super().__getattr__(attr)
  
  def __setattr__(self, attr, val):
    if attr == "theta":
      r = self.radius
      self.x = r * cos(val)
      self.y = r * sin(val)
    elif attr == "radius":
      theta = self.theta
      self.x = val * cos(theta)
      self.y = val * sin(theta)
    else:
      super().__setattr__(attr, val)

  def __getitem__(self, i):
    if i == 0:
      return self.x
    if i == 1:
      return self.y
    raise IndexError("index out of range")

  def distance_to(self, other):
    other = Point(other)
    return sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2)

  def lerp_to(self, other, fraction):
    x = fraction * (other.x - self.x) + self.x
    y = fraction * (other.y - self.y) + self.y
    return Point(x,y)
  
  def lerp_distance(self, other, distance):
    totalDistance = self.distance_to(other)
    return self.lerp_to(other, distance/totalDistance)

  def __neg__(self):
    return Point(-self.x, -self.y)

  def __str__(self):
    return "(%f, %f)" % (self.x, self.y)
  
  def __repr__(self):
    return str(self)

  def __add__(self, oth):
    oth = Point(oth)
    return Point(self.x + oth.x, self.y + oth.y)

  def __sub__(self, oth):
    oth = Point(oth)
    return Point(self.x - oth.x, self.y - oth.y)

  def __truediv__(self, scalar):
    return Point(self.x/scalar, self.y/scalar)
  
  def __eq__(self, oth):
    try:
      return self.x == oth.x and self.y == oth.y
    except AttributeError:
      return False
      
  def __lt__(self, oth):
    return (self.x, self.y) < (oth.x, oth.y)

  def __complex__(self):
    return self.x + self.y * 1j

###############################################################################

def pretty(obj):
  if type(obj) is HashableItem:
    obj = obj.item
  assert obj.this is not None, "Calling pretty on item without _this_: %s" % (obj, )
  if type(obj) is list or type(obj) is tuple or type(obj) is set:
    pretties = [pretty(it) for it in obj]
    return "[{}]".format(", ".join(pretties))
  elif isPad(obj):
    return "<pcbnew.PAD {} of fp '{}' ({}) at {} [this={}]".format(obj.GetPadName(), obj.GetParentFootprint().GetReference(), obj.GetNetname(), Point(obj.GetPosition()), obj.this)
  elif isTrack(obj):
    return "<pcbnew.PCB_TRACK ({}) at {} => {} [this={}]".format(obj.GetNetname(), Point(obj.GetStart()), Point(obj.GetEnd()), obj.this)
  elif type(obj) is pcbnew.PCB_VIA:
    return "<pcbnew.PCB_VIA ({}) at {} [this={}]".format(obj.GetNetname(), Point(obj.GetPosition()), obj.this)
  elif type(obj) is pcbnew.FOOTPRINT:
    return "<pcbnew.FOOTPRINT '{}' at {}>", obj.GetReference(), Point(obj.GetPosition())
  elif type(obj) is pcbnew.BOARD_ITEM_CONTAINER:
    return str(obj)
  elif obj is None:
    return "pretty(None)"
  else:
    raise TypeError(type(obj))

###############################################################################

class Timing(object):
  @classmethod
  def linear(cls, time, start, change, duration):
    return start + change * time/duration
  
  @classmethod
  def easeInOutCubic(cls, time, start, change, duration): 
    t = time / (duration/2)
    return change/2*t*t*t + start if t < 1 else change/2*((t-2)*(t-2)*(t-2) + 2) + start;
    
  @classmethod
  def easeOutCubic(cls, time, start, change, duration):
    time = time/duration
    t2 = time-1
    return change*(t2*t2*t2 + 1) + start

  @classmethod
  def easeInCubic(cls, time, start, change, duration):
    time = time/duration
    return change * time*time*time + start

## TraceNet #############################################################################

# sort of hacky wrapper around pcbnew objects that supported hashing in kicad 7, but not kicad 8. 
class HashableItem:
  def __init__(self, item):
    if type(item) is HashableItem:
      item = item.item
    assert type(item) == pcbnew.PAD or type(item) == pcbnew.PCB_TRACK or type(item) == pcbnew.PCB_VIA, "HashableItem supports pcbnew PAD, PCB_TRACK, and PCB_VIA, not {}".format(type(item))
    self.item = item
  
  def __hash__(self):
    if type(self.item) == pcbnew.PAD:
      return hash((tuple(self.item.GetPosition()), self.item.GetNetname(), self.item.GetParentFootprint().GetReference(), self.item.GetName()))
    elif type(self.item) == pcbnew.PCB_TRACK:
      return hash((tuple(self.item.GetStart()), tuple(self.item.GetEnd()), self.item.GetNetname(), self.item.GetLayerName()))
    elif type(self.item) == pcbnew.PCB_VIA:
      return hash((tuple(self.item.GetPosition()), self.item.GetNetname(), tuple(self.item.GetLayerSet().CuStack())))
  
  def __eq__(self, other):
    if isinstance(other, HashableItem):
      return self.item == other.item
    return False
  
  def __repr__(self):
    return "<HashableItem wrapping {}>".format(pretty(self.item))
  
  def __getattr__(self, attr):
    return getattr(self.item, attr)
  
def isTrack(t):
  return type(t) is pcbnew.PCB_TRACK or (type(t) is HashableItem and type(t.item) is pcbnew.PCB_TRACK)

def isPad(t):
  return type(t) is pcbnew.PAD or (type(t) is HashableItem and type(t.item) is pcbnew.PAD)

def isVia(t):
  return type(t) is pcbnew.PCB_VIA or (type(t) is HashableItem and type(t.item) is pcbnew.PCB_VIA)

class TraceNet(object):
  def __init__(self, tracks, pads):
    self.tracks = [HashableItem(t) for t in tracks]
    self.pads = [HashableItem(p) for p in pads]
    self.netname = None
    self.layers = set()
    
    for t in self.tracks:
      netname = t.GetNetname()
      # assert self.netname is None or self.netname == netname, "TraceNet tracks across nets {} and {}".format(self.netname, netname)
      if self.netname is not None and self.netname != netname:
        print("WARNING: TraceNet pads across nets {} and {}".format(self.netname, netname))
      self.netname = netname
      self.layers.add(t.GetLayerName())
    for p in self.pads:
      netname = p.GetNetname()
      # assert self.netname is None or self.netname == netname, "TraceNet tracks across nets {} and {}".format(self.netname, netname)
      if self.netname is not None and self.netname != netname:
        print("WARNING: TraceNet pads across nets {} and {}".format(self.netname, netname))
      self.netname = netname
      # try:
      ref = p.GetParentFootprint().GetReference()
      # except AttributeError:
      #   ref = str(p.GetParentFootprint()) + str(Point(p.GetPosition()))
      padLayerHasTrack = False
      for layer in self.layers:
        if p.IsOnLayer(p.GetBoard().GetLayerID(layer)):
          padLayerHasTrack = True
          break
      assert padLayerHasTrack, "Expected pad ref {}, net {}, to be on a layer connected to its tracks, layers={}".format(ref, netname, self.layers)

  def totalLength(self):
    dist = 0
    for t in self.tracks:
      dist += Point(t.GetStart()).distance_to(t.GetEnd())
    return dist
  
  def traceCount(self):
    return len([v for v in self.tracks if isTrack(v)])

  def viaCount(self):
    return len([v for v in self.tracks if isVia(v)])
  
  def padCount(self):
    return len(self.pads)
  
  def __lt__(self, oth):
    return self.netname < oth.netname or (self.netname == oth.netname and self.totalLength() < oth.totalLength())
  
  def __str__(self):
    return "<TraceNet {}: {}t{}v".format(self.netname, self.traceCount(), self.viaCount())

## KiCadPCB #############################################################################
  
class KiCadPCB(object):
  def __init__(self, path):
    self.pcb_path = os.path.abspath(path)
    self.board = pcbnew.LoadBoard(self.pcb_path)
    self.resetCache()

  def save(self):
    print("Saving...")
    backup_path = self.pcb_path + ".layoutbak"
    if os.path.exists(backup_path):
      os.unlink(backup_path)
    os.rename(self.pcb_path, backup_path)
    print("  Backing up '%s' to '%s'..." % (self.pcb_path, backup_path))
    assert self.board.Save(self.pcb_path)
    print("Saved!")

  def resetCache(self):
    self.tracks = list(HashableItem(t) for t in self.board.GetTracks())
    self.pads = list(HashableItem(p) for p in self.board.GetPads())
    self._sortedTracks = None
    self._sortedPads = None

    self.numlayers = pcbnew.PCB_LAYER_ID_COUNT

    self.layertable = {}
    for i in range(self.numlayers):
      self.layertable[self.board.GetLayerName(i)] = i

    self.netcodes = self.board.GetNetsByNetcode()
    self.netnames = {}  
    for netcode, net in self.netcodes.items():
      self.netnames[net.GetNetname()] = net
    
  def removeItemFromCache(self, item):
    # purely to help performance so we don't have to reset the cache repeatedly
    def indexesMatching(enum, pred):
      matches = []
      for i, item in enumerate(enum):
        if pred(item):
          matches.append(i)
      return matches
    assert type(item) is HashableItem
    if self.tracks and (isTrack(item) or isVia(item)):
      self.tracks.remove(item)
      if self._sortedTracks:
        for i in reversed(indexesMatching(self._sortedTracks, lambda st: st[1] == item)):
          del self._sortedTracks[i]
        
    elif self.pads and isPad(item):
      self.pads.remove(item)
      if self._sortedPads:
        for i in reversed(indexesMatching(self._sortedPads, lambda st: st[1] == item)):
          del self._sortedPads[i]
    else:
      assert False, "what is item? %s" % str(item)

  def deleteEdgeCuts(self):
    self.deleteAllDrawings(layer='Edge.Cuts')

  def deleteAllTraces(self):
    tracks = self.board.GetTracks()
    for t in tracks:
      print("Deleting track {}".format(t))
      self.board.Delete(t)
  
  def deleteAllDrawings(self, layer=None):
    for drawing in self.board.GetDrawings():
        if layer is None or layer == drawing.GetLayerName():
          print("Deleting drawing {}".format(drawing))
          self.board.Delete(drawing)
  
  def deleteFootprintConnectedItems(self, fp):
    for pad in fp.Pads():
      print("  Finding traces connected to {} pad {} net {}...".format(fp.GetReference(), pad.GetPadName(), pad.GetNetname()))
      traceNet = self.getTraceNet(pad)
      print("  Deleting old tracks ({} items) for fp {} pad {} net {}".format(len(traceNet.tracks), fp.GetReference(), pad.GetPadName(), pad.GetNetname()))
      for item in traceNet.tracks:
        assert type(item) is HashableItem
        print ("    Deleting trace item", pretty(item))
        self.board.Delete(item.item)
        self.removeItemFromCache(item)

  def deleteStrayVias(self):
    print("Deleting stray vias if any...")
    tracks = self.sortedTracks()
    for pt,t in tracks:
      if isVia(t):
        traceNet = self.getTraceNet(t)
        if traceNet.traceCount() == 0:
          print("  Deleting stray via at {}".format(Point(t.GetPosition())))
          self.board.Delete(t.item)
          self.removeItemFromCache(t)

  # == Info ===
  def dumpObjects(self, layer=None):
    print("Dumping All Objects", "" if (type(layer)!= str) else "on layer %s"%layer)
    def dumpiflayer(obj):
      if type(layer) != str or obj.GetLayerName() == layer:
        print(obj)
        for attrname in dir(obj):
          attr = getattr(obj, attrname)
          try:
            print(" ", attrname, type(attr), end='')
            if attrname in ("DeleteStructure", "GetCorners"):
              print("  ->  ", "skipping due to potential segfault")
              continue
            sys.stdout.flush()
            print("  ->  ", attr() if callable(attr) else str(attr))
          except TypeError:
            print("  ->  ", "method needs arguments")
    for obj in self.board.GetTracks():
      dumpiflayer(obj)
    for obj in self.board.GetFootprints():
      dumpiflayer(obj)
    for obj in self.board.GetDrawings():
      dumpiflayer(obj)
    for obj in self.board.Zones():
      dumpiflayer(obj)
    # for obj in self.board.GetMarkers():
    #   dumpiflayer(obj)
      
  def dumpStats(self):
    from operator import itemgetter
    
    tracks = set(self.tracks)

    traceNets = []
    while len(tracks) > 0:
      t = tracks.pop()
      tn = self.getTraceNet(t)
      if len(tn.pads) == 0 and (len(tn.tracks) != 1 or not isVia(list(tn.tracks)[0])):
        print("  Found a connected group with no pads?")
        # FIXME: this finds orphans sometimes, but usually just trace groups that are actually connected by zone fills
        for t in tn.tracks:
          print("    track {} from {} to {} on net {}".format(type(t), Point(t.GetStart()), Point(t.GetEnd()), t.GetNetname()))
      print(".",end="",flush=True)
      for t in tn.tracks:
        tracks.discard(t)
      traceNets.append(tn)
    
    traceNets = sorted(traceNets)

    # stats TODO:
    # * zone connections
    # * draw as a table showing stats per Net as well as totals
    # * diff against a git commit

    print("")
    print("== Stats =======================================================================")
    print("Number of connected trace/via/pad groups (modulo zone-connections): {}".format(len(traceNets)))
    print("-"*80)

    for tn in traceNets:
      print("Trace group {} with {} trace segments of total length {:.3f} and {} vias, connecting {} pads".format(tn.netname, tn.traceCount(), tn.totalLength(), tn.viaCount(), tn.padCount()))

    print("-- Length --")
    trackLengths = [(tn.totalLength(), tn) for tn in traceNets]
    themax = max(trackLengths, key=itemgetter(0))
    print("Longest connected track group: {:.3f}mm on net {}".format(themax[0], themax[1].netname))
    
    totalTrackLength = sum(tl[0] for tl in trackLengths)
    print("Track length of all tracks: {:.3f}mm".format(totalTrackLength))
    
    print("-- Vias --")
    traceNetVias = [(tn.viaCount(), tn) for tn in traceNets]
    totalVias = sum(tnv[0] for tnv in traceNetVias)
    print("Average number of vias per mm of track: {:.4f}".format(totalVias/totalTrackLength))

    traceNetViasExcluding = [(tn.viaCount(), tn) for tn in traceNets if not tn.netname.startswith('GND') and tn.netname != '-BATT']
    totalViasExcluding = sum(tnv[0] for tnv in traceNetViasExcluding)
    if totalViasExcluding > 0:
      totalTrackLengthExcluding = sum(tl[0] for tl in trackLengths if not tl[1].netname.startswith('GND') and tl[1].netname != '-BATT')
      print("Average number of vias per mm of track (excluding GND* and -BATT): {:.4f}".format(totalViasExcluding/totalTrackLengthExcluding))

    for tn1 in traceNets:
      for tn2 in traceNets:
        if tn1 != tn2:
          assert set(tn1.tracks).isdisjoint(tn2.tracks), "Overlapping groups: maybe failed to follow a connected trace path: {} and {}".format(tn1, tn2)

  def itemLayersIntersect(self, item1, item2):
    parent = item1.GetParentFootprint() if item1.GetParentFootprint() else item1.GetParent()
    if parent.GetFlags() & pcbnew.STRUCT_DELETED:
      print("    * Skipping intersect check on item with deleted parent", pretty(parent))
      return False
    if parent.GetFlags() & pcbnew.STRUCT_DELETED:
      print("    * Skipping intersect check on item with deleted parent", pretty(item2))
      return False
    return not set(item1.GetLayerSet().Seq()).isdisjoint(set(item2.GetLayerSet().Seq()))

  def listIndexesNearPoint(self, indexedList, pt, item, hitBox = 0.1):#mm
    """ given a sorted list of the form [(point, item), ...] and an item at pt, find the indexes of all other tracks,vias,pads within hitbox (mm) of the given point on layers intersecting the given item """

    def hitTestAtIndex(pt, srcItem, index):
      # FIXME: I'm not sure why this was here? getTraceNet contain the assumption that searching for a pad will find exactly one other pad, which should be the same pad. 
      # if srcItem == indexedList[index][1]:
      #   print("skipping self intersection")
      #   return False

      assert indexedList[index][1].this is not None, "hitTestAtIndex given index refers to deleted item at index %i, %s" % (index, str(indexedList[index][1]))
      pointHit = pt.distance_to(indexedList[index][0]) < hitBox
      # TODO: any benefit to using kicad's HitTest?
      # pointHit = indexedList[index][1].HitTest(pt.vector2i(), int(hitBox * IU_PER_MM))
      return pointHit and self.itemLayersIntersect(srcItem, indexedList[index][1])

    def clusterAt(index):
      # the binary search should find a cluster of items all within hit range
      bsHit = hitTestAtIndex(pt, item, index)
      cluster = [index] if bsHit else []

      for l in range(index-1, -1, -1):
        if hitTestAtIndex(pt, item, l):
          cluster.insert(0, l)
        elif abs(pt.x - indexedList[l][0].x) > hitBox:
          break
      for r in range(index+1, len(indexedList)):
        if hitTestAtIndex(pt, item, r):
          cluster.append(r)
        elif abs(pt.x - indexedList[r][0].x) > hitBox:
          break
      return cluster

    startIndex = 0
    endIndex = len(indexedList)-1
    while endIndex >= startIndex:
      index = int(startIndex + (endIndex - startIndex)/2)
      # if isPad(indexedList[startIndex][1]):
      #   print("    binary search [{},{}] is {} <--to--> {}".format(startIndex, endIndex, pretty(indexedList[startIndex][1]), pretty(indexedList[endIndex][1])))
      #   print("    considering index {} is {}".format(index, pretty(indexedList[index][1])))
      if pt == indexedList[index][0] or hitTestAtIndex(pt, item, index):
        return clusterAt(index)
      if pt < indexedList[index][0]:
        endIndex = index-1
      else:
        startIndex = index+1
    return []
  
  def sortedItemsNear(self, item, sortedList):
    if isTrack(item):
      nearStart = self.listIndexesNearPoint(sortedList, Point(item.GetStart()), item)
      nearEnd = self.listIndexesNearPoint(sortedList, Point(item.GetEnd()), item)
      return [sortedList[i][1] for i in nearStart] + [sortedList[i][1] for i in nearEnd]
    elif isPad(item) or isVia(item):
      nearby = self.listIndexesNearPoint(sortedList, Point(item.GetPosition()), item)
      return [sortedList[i][1] for i in nearby]
    else:
      raise TypeError(item)

  def sortedTracks(self):
    from operator import itemgetter
    if self._sortedTracks is None:
      sortedTracks = [(Point(t.GetStart()), t) for t in self.tracks]
      sortedTracks.extend([(Point(t.GetEnd()), t) for t in self.tracks if isTrack(t)])
      self._sortedTracks = list(sorted(sortedTracks, key=itemgetter(0)))
    return self._sortedTracks
  
  def sortedPads(self):
    from operator import itemgetter
    if self._sortedPads is None:
      sortedPads = [(Point(p.GetPosition()), p) for p in self.pads]
      self._sortedPads = list(sorted(sortedPads, key=itemgetter(0)))
    return self._sortedPads

  def getTraceNet(self, item):
    """ 
      return a TraceNet object of all tracks, pads and vias, connected to the given track 
      item should be a PCB_TRACK, PCB_VIA, or PAD
      does not consider zone connections
    """
    #FIXME: need zone intersection processing
    #FIXME: Warn or handle it if we're passed an item that is not already in the sorted lists, since it will result in duplicates
    item = HashableItem(item)
    if isPad(item):
      pads = self.sortedItemsNear(item, self.sortedPads())
      assert len(pads)==1, "expect 1 pad connection but have {} for pad {} -> {}".format(len(pads), item.GetParentFootprint().GetReference(), item.GetName())
      if item != pads[0]:
        # we may be given a Pad object that is the same pad but a different reference to it
        assert(item.GetPadName() == pads[0].GetPadName())
        assert(item.GetParentFootprint().GetReference() == pads[0].GetParentFootprint().GetReference())
        item = pads[0]

    trackSet = set()
    totalVisited = 0
    unvisited = set([HashableItem(item)])
    while len(unvisited) > 0:
      totalVisited+=1
      t = unvisited.pop()
      if not isPad(t):
        trackSet.add(t)
      nearbyTracks = self.sortedItemsNear(t, self.sortedTracks())
      for nt in nearbyTracks:
        if nt not in trackSet:
          unvisited.add(nt)
    
    padSet = set()
    for t in trackSet:
      for p in self.sortedItemsNear(t, self.sortedPads()):
        padSet.add(p)
    if len(trackSet)>0:
      if isPad(item):
        # this assertion is firing in some cases?
        assert item in padSet

    return TraceNet(trackSet, padSet)

  # == Drawing Tools ==
  def draw_segment(self, start, end, layer='F.Silkscreen', width=0.15):
    print("LINE from (%f, %f) to (%f, %f) on %s" % (start.x, start.y, end.x, end.y, layer))
    line = pcbnew.PCB_SHAPE()
    self.board.Add(line)
    line.SetShape(pcbnew.S_SEGMENT)
    line.SetStart(start.vector2i())
    line.SetEnd(end.vector2i())
    line.SetLayer(self.layertable[layer])
    line.SetWidth(int(width * IU_PER_MM))
    line.SetLocked(args.lock)
    return line
  
  def draw_circle(self, center, radius, layer='F.Silkscreen', width=0.15):
      print("CIRCLE at {} radius {} on {}".format(center, radius, layer))
      circle = pcbnew.PCB_SHAPE()
      self.board.Add(circle)
      circle.SetShape(pcbnew.SHAPE_T_CIRCLE)
      circle.SetCenter(center.vector2i())
      circle.SetArcAngleAndEnd(pcbnew.EDA_ANGLE(360, pcbnew.DEGREES_T))
      circle.SetEnd((center + Point(radius,0)).vector2i()) # Sets radius.. somehow
      circle.SetLayer(self.layertable[layer])
      circle.SetWidth(int(width * IU_PER_MM))
      circle.SetLocked(args.lock)
      return circle

  def draw_arc(self, start, center=None, end=None, angle=None, layer='F.Silkscreen', width=0.15):
      print("Drawing arc with start: {}, center: {}, end: {}, angle: {}".format(start, center, end, angle))
      arc = pcbnew.PCB_SHAPE(self.board)
      arc.SetStart(start.vector2i())
      arc.SetShape(pcbnew.SHAPE_T_ARC)
      if center is not None:
        arc.SetCenter(center.vector2i())
      # either end or angle may be specified
      if end is not None:
        arc.SetEnd(end.vector2i())
      if angle is not None:
        arc.SetArcAngleAndEnd(pcbnew.EDA_ANGLE(angle, pcbnew.RADIANS_T))
      
      arc.SetLayer(self.layertable[layer])
      arc.SetWidth(int(width * IU_PER_MM))
      self.board.Add(arc)
      arc.SetLocked(args.lock)
      return arc

  def add_copper_trace(self, start, end, net, layer="F.Cu", width = 0.2):
    start = Point(start)
    end = Point(end)
    track = pcbnew.PCB_TRACK(self.board)
    track.SetStart(start.vector2i())
    track.SetEnd(end.vector2i())
    track.SetLayer(self.layertable[layer])
    track.SetWidth(int(width * IU_PER_MM))
    track.SetNet(net)
    track.SetLocked(args.lock)
    self.board.Add(track)
    print("  copper from {} to {} on net {} layer {}".format(start, end, net.GetNetname(), layer))
    return track

  def add_via(self, position, net, fromLayer="F.Cu", toLayer="B.Cu"):
    position = Point(position)
    print("VIA at {} on net {} at {}".format(position, net.GetNetname(), str(position)))
    via = pcbnew.PCB_VIA(self.board)
    via.SetPosition(position.vector2i())
    if fromLayer != "F.Cu" or toLayer != "B.Cu":
      via.SetViaType(pcbnew.VIATYPE_BLIND_BURIED)
      via.SetLayerPair(self.layertable[fromLayer], self.layertable[toLayer])
    via.SetDrill(int(BoardConstraints.minDrill * IU_PER_MM))
    via.SetWidth(int((BoardConstraints.minDrill + 2*BoardConstraints.annularRing) * IU_PER_MM))
    via.SetNet(net)
    via.SetLocked(args.lock)
    self.board.Add(via)


## TraceBuilder #############################################################################


class TraceBuilder(object):
  defaultTraceWith = 0.2
  class Via(object):
    def __init__(self, pt):
      self.point = pt
    def __repr__(self):
      return "TraceBuilder.Via({})".format(self.point)
  class Trace(object):
    def __init__(self, pt):
      self.point = pt
    def __repr__(self):
      return "TraceBuilder.Trace({})".format(self.point)
  
  def multitrace(self, kicadpcb, points, net, startlayer):
    front = (startlayer=="F.Cu")
    lastPoint = points[0].point
    for p in points[1:]:
      isVia = type(p) is self.Via
      p = p.point
      if p.distance_to(lastPoint) > 0.0001:
        # then only draw traces if they are long enough
        # TODO: handle vias to hidden layers
        kicadpcb.add_copper_trace(lastPoint, p, net, "F.Cu" if front else "B.Cu", width=self.defaultTraceWith)
        lastPoint = p
      if isVia:
        kicadpcb.add_via(p, net)
        front = not front
    if len(points) == 1:
      p = points[0]
      isVia = type(p) is self.Via
      if isVia:
        kicadpcb.add_via(p.point, net)
  
  def __init__(self, startPad=None, startlayer="F.Cu"):
    self.points = []
    self.startlayer = startlayer
    self._startPad = None
    if startPad is None:
      self.addPoint(Point(0,0))
    elif type(startPad) == Point:
      self.addPoint(startPad)
    else:
      self._startPad = startPad
      self.addPoint(Point(startPad.GetPosition()))
  
  def addPoint(self, p):
    if len(self.points) == 0:
      self.startPoint = p # save off start point as a rotation reference in case we trim the path later
    if type(p)==self.Via or type(p)==self.Trace:
      self.points.append(p)
    else:  
      self.points.append(self.Trace(Point(p)))

  @property
  def startPad(self):
    return self._startPad

  @startPad.setter
  def startPad(self, newStartPad):
      if self.startPad is not None:
        self.rotate(self.startPad.GetParentFootprint().GetOrientation().AsRadians())
      self.rotate(-newStartPad.GetParentFootprint().GetOrientation().AsRadians())
      prevStart = Point(self.startPad.GetPosition()) if self.startPad else Point(0,0)
      self.translate(Point(newStartPad.GetPosition())-prevStart)
      self._startPad = newStartPad
  
  def draw(self, kicadpcb):
    self.multitrace(kicadpcb, self.points, self._startPad.GetNet(), self.startlayer)
    return self
  
  # path manipulation

  def useAsPivot(self):
    """ Use the last point in this TraceBuilder as a waypoint in another tracebuilder """
    self._startPad = None
    self.points = [self.Trace(self.lastPoint())]
    return self

  def translate(self, v):
    for i, p in enumerate(self.points):
      self.points[i] = type(p)(p.point + v)

  def rotate(self, theta):
    center = self.startPoint
    self.translate(-center)
    for i, p in enumerate(self.points):
      x = p.point.x
      y = p.point.y
      p.point.x = x*cos(theta) - y*sin(theta)
      p.point.y = x*sin(theta) + y*cos(theta)
      self.points[i] = p
    self.translate(center)

  def __getitem__(self, val):
    # support getting a sliced TraceBuilder
    from copy import deepcopy
    oth = deepcopy(self)
    if type(val) is not slice:
      if val<0:
        val += len(oth.points)
      val = slice(val, val+1)
    oth.points = oth.points[val]
    return oth

  #### point manipulation

  def lastPoint(self):
    for t in self.points[::-1]:
      return t.point
      
  def angleConst(self, distance, angle):
    self.addPoint(self.lastPoint().polar_translated(distance, angle))
    return self
  
  def via(self):
    self.points.append(self.Via(self.lastPoint()))
    return self
  
  def xconst(self, x):
    self.addPoint(self.lastPoint() + (x,0))
    return self
  
  def yconst(self, y):
    self.addPoint(self.lastPoint() + (0, y))
    return self
  
  def octPathCloseTo(self, end):
    """ Close the path to end pcbnew.PAD or TraceBuilder using a diagonal followed by a cardinal trace """
    othertb = None
    if type(end) is TraceBuilder:
      othertb = end
      end = end.lastPoint()
    try:
      end = Point(end.GetPosition())
    except AttributeError:
      pass
    # diagonal path to a cardinal line, then close it
    lp = self.lastPoint()
    distance = end - lp
    if abs(distance.x) < abs(distance.y):
      # x closer
      self.addPoint(lp + (distance.x, sign(distance.y)*abs(distance.x)))
    else:
      # y closer
      self.addPoint(lp + (sign(distance.x)*abs(distance.y), distance.y))
    if othertb:
      for p in othertb.points[::-1]:
        self.addPoint(p)
    else: 
      self.addPoint(end)
    return self

  def joinWithCardialAtSplitRatio(self, ratio, other):
    """ Close the path to other TraceBuilder using a cardinal-direction bar at ratio between the height-gap or width-gap (whichever is less) of the last points on each TraceBuilder """
    assert(0 <= ratio <= 1)
    distance = other.lastPoint() - self.lastPoint()
    if abs(distance.x) < abs(distance.y):
      # x closer
      self.addPoint(self.lastPoint() + (distance.x*ratio, sign(distance.y)*abs(distance.x)*ratio))
      self.addPoint(other.lastPoint() - (distance.x*(1-ratio), sign(distance.y)*abs(distance.x)*(1-ratio)))
    else:
      # y closer
      self.addPoint(self.lastPoint() + (sign(distance.x)*abs(distance.y)*ratio, distance.y*ratio))
      self.addPoint(other.lastPoint() - (sign(distance.x)*abs(distance.y)*(1-ratio), distance.y*(1-ratio)))
    for p in other.points[::-1]:
      self.addPoint(p)
    return self

## Pixel #############################################################################

class Pixel(object):
  footprintName = ""
  connectionMap = {}
  powerPin = None
  groundPin = None
  powerConnect = None
  groundConnect = None
  def __init__(self, name, powerPin, groundPin, dataConnections, powerConnect, groundConnect):
    self.footprintName = name
    self.powerPin = powerPin
    self.groundPin = groundPin
    self.connectionMap = dataConnections
    self.powerConnect = powerConnect
    self.groundConnect = groundConnect

SK9822EC20 = Pixel("LED-SK9822-EC20", powerPin=5, groundPin=2, dataConnections={1:3, 6:4}, 
                   powerConnect=TraceBuilder().angleConst(0.5, 0).via(), 
                   groundConnect=TraceBuilder().angleConst(0.5, pi).via())
WS2812B1010 = Pixel("ws2812b_1010", powerPin=2, groundPin=3, dataConnections={1:4}, 
                    powerConnect=TraceBuilder().angleConst(0.5, 0).via(), 
                    groundConnect=TraceBuilder().angleConst(0.5, 3*pi/4).via())

## PCBLayout #############################################################################

class PCBLayout(object):
  center = Point(100,100)
  edge_cut_line_thickness = 0.05
  
  zoneMinThickness = 0.15
  zoneClearance = 0.2

  ####

  def __init__(self, path, pixelType):
    self.kicadpcb = KiCadPCB(path)
    self.board = self.kicadpcb.board
    self.pixelType = pixelType

  seriesPixelPrevious = {}
  pixel_prototype = None
  seriesPixelCounts = {}

  zones = {}

  def addZonePoint(self, point, zoneName, netName, layers, priority=0):
    if zoneName in self.zones:
      zone = self.zones[zoneName]
      outline = zone.Outline()
      assert(outline.FullPointCount() > 0)
    else:
      print("Create Zone {} with net {} on layers {}".format(zoneName, netName, layers))
      zone = pcbnew.ZONE(self.board)
      self.zones[zoneName] = zone
      zone.SetZoneName(zoneName)
      zone.SetMinThickness(int(self.zoneMinThickness*IU_PER_MM))
      zone.SetLocalClearance(int(self.zoneClearance*IU_PER_MM))
      zone.SetAssignedPriority(priority)
      # zone.SetZoneConnection(pcbnew.ZONE_CONNECTION_FULL)
      # zone.SetThermalReliefSpokeWidth(0.4)
      zone.SetThermalReliefGap(int(0.4*IU_PER_MM))
      
      zone.SetLocked(args.lock)
      if type(layers) is str:
        zone.SetLayer(self.kicadpcb.layertable[layers])
      else:
        ls = pcbnew.LSET()
        for layer in layers:
          ls.AddLayer(self.kicadpcb.layertable[layer])
        zone.SetLayerSet(ls)
      if netName not in self.kicadpcb.netnames:
        net = pcbnew.NETINFO_ITEM(self.board, netName)
        # FIXME: I don't think this board.Add(net) is working right, but AppendNet was removed in i think kicad 8
        self.board.Add(net)
        self.kicadpcb.netnames[netName] = net
      zone.SetNet(self.kicadpcb.netnames[netName])
      outline = zone.Outline()
      outline.NewOutline()
      self.board.Add(zone)

    print("  Point for Zone {} at {}".format(zoneName, point))
    outline.Append((self.center+point).vector2i())
    return zone

  def drawSegment(self, start, end, layer='F.Silkscreen', width=0.15):
    start = self.center+Point(start)
    end = self.center+Point(end)
    return self.kicadpcb.draw_segment(start, end, layer, width)

  def drawArcFromPoints(self, start, center, end, layer='F.Silkscreen', width=0.15):
    start = self.center+Point(start)
    center = self.center+Point(center)
    end = self.center+Point(end)
    return self.kicadpcb.draw_arc(start, center, end=end, angle=None, layer=layer, width=width)

  def drawArcFromAngle(self, start, center, angle, layer='F.Silkscreen', width=0.15):
    start = self.center+Point(start)
    center = self.center+Point(center)
    return self.kicadpcb.draw_arc(start, center, end=None, angle=angle, layer=layer, width=width)    

  def drawCircle(self, center, radius, layer='F.Silkscreen', width=0.15):
    center = self.center+Point(center)
    return self.kicadpcb.draw_circle(center, radius, layer=layer, width=width)

  def drawZoneCircle(self, center, radius, zoneName, netName, layers, priority=0):
    segmentPerMM = 2
    segments = round(2*pi*radius*segmentPerMM)
    for i in range(segments):
      t = 2*pi*i/segments
      p = center + Point(radius*cos(t), radius*sin(t))
      self.addZonePoint(p, zoneName, netName, layers, priority)

  def drawPoly(self, polySpec):
    def regularPolygonVertices(n_sides, radius):
      angle = 2*pi / n_sides
      vertices = []
      for i in range(n_sides):
        theta = angle * i + 2*pi/n_sides/2
        x = radius * cos(theta)
        y = radius * sin(theta)
        vertices.append((x, y))
      return vertices

    sides, radius, layer = polySpec.split(',')
    sides = int(sides)
    radius = float(radius)

    vertices = regularPolygonVertices(sides,radius)
    for i in range(len(vertices)):
      x1,y1 = vertices[i]
      x2,y2 = vertices[i+1-len(vertices)]
      assert(x1!= x2 or y1 != y2)
      self.drawSegment((x1, y1), (x2, y2), layer)

  def drawWave(self, waveSpec):
    import cmath
    def reorder(a,b):
      if a > b:
        return b,a
      return a,b

    startx,starty,endx,endy,cycles,amplitude,segments,layer = waveSpec.split(',')
    startx,starty,endx,endy,cycles,amplitude = mkfloat(startx,starty,endx,endy,cycles,amplitude)
    segments = int(segments)
    cycles = 2*pi*cycles
    p1,p2 = reorder((startx,starty), (endx, endy)) # reduce to two quandrants
    p1 = Point(p1)
    p2 = Point(p2)

    print("Draw Wave from (",startx, ",",starty,") to (",endx,",",endy,")")

    prevPoint = None
    for i in range(segments+1):
      xp = i / float(segments) * (p2.x-p1.x)
      yp = -amplitude * cos(i / float(segments) * cycles)
      angle = atan((p2.y-p1.y)/(p2.x-p1.x))

      rotated = Point((xp+yp*1j) * cmath.exp(angle*1j)) + p1
      if prevPoint is not None:
        self.drawSegment(prevPoint,rotated,layer,0.05)
      prevPoint = rotated

  def drawEllipse(self, ellipseSpec):
    """ centerx,centery,xradius,yradius,segments,layer """
    centerx,centery,xradius,yradius,rotation,segments,layer = ellipseSpec.split(',')
    centerx,centery,xradius,yradius,rotation,segments = mkfloat(centerx,centery,xradius,yradius,rotation,segments)
    segments = int(segments)

    prevPoint = None
    for s in range(segments+1):
      x = xradius * cos(2*pi*s/segments)
      y = yradius * sin(2*pi*s/segments)
      p = Point(centerx, centery) + Point(x,y).rotate_about(rotation, Point(0,0))
      if prevPoint is not None:
        self.drawSegment(prevPoint,p,layer,0.05)
      prevPoint = p

  ### ------------------------------------------------------- ###

  def connectPixels(self, fromPixel, toPixel, connectorFunction=None):
    print("Connect pixels", fromPixel.GetReference(), "(orientation:", fromPixel.GetOrientation().AsDegrees(), fromPixel.GetOrientation().AsRadians(), ") ->", toPixel.GetReference(), "(orientation:", toPixel.GetOrientation().AsDegrees(), toPixel.GetOrientation().AsRadians(),")")
    if not args.skip_traces:
      # Add data tracks from the previous pixel
      alternate = -1 if int(str(fromPixel.GetReference()).strip("D")) % 2 else 1
      for prev_pad in fromPixel.Pads():
        if int(prev_pad.GetPadName()) in self.pixelType.connectionMap:
          # then connect the two pads
          for pad in toPixel.Pads():
            if int(pad.GetPadName()) == self.pixelType.connectionMap[int(prev_pad.GetPadName())]:
              start = Point(prev_pad.GetPosition())
              end = Point(pad.GetPosition())
              if connectorFunction:
                connectorFunction(prev_pad, pad)
              else:
                self.kicadpcb.add_copper_trace(start, end, pad.GetNet(), fromPixel.GetLayerName())
  
  def insertFootprint(self, footprintNameOrPrototype, reference, point, orientation, layer):
    onBack = (layer =="B.Cu")
    point = self.center+point
    for fp in self.kicadpcb.board.GetFootprints():
      if fp.GetReference() == reference:
        print("Removing old footprint for", reference)
        self.kicadpcb.board.Delete(fp)
        break
    if type(footprintNameOrPrototype) is str:
      fp = pcbnew.FootprintLoad(str(Path(__file__).parent.parent.joinpath('kicad_footprints.pretty').resolve()), footprintNameOrPrototype)
      if not fp:
        lib, name = footprintNameOrPrototype.split(":")
        fp = pcbnew.FootprintLoad(lib, name)
      assert fp, "Failed to load footprint {}".format(footprintNameOrPrototype)
    else:
      fp = pcbnew.FOOTPRINT(footprintNameOrPrototype)
    
    print("** Placing footprint reference {} at point {} orientation {} layer {}".format(reference, point, orientation, layer))
    print("    {}".format(fp))
    if onBack:
      # Here we insert the footprint on the front, then Flip() it to the back, rather than try and natively insert it on the back, which doesn't work.
      layer = "F.Cu"

    fp.SetLayer(self.kicadpcb.layertable[layer])
    fp.SetPosition(point.vector2i())
    fp.SetOrientation(pcbnew.EDA_ANGLE(orientation, pcbnew.RADIANS_T))
    fp.SetReference(reference)
    self.kicadpcb.board.Add(fp)
    fp.Reference().SetVisible(False)
    fp.SetLocked(args.lock)
    if onBack:
      fp.Flip(point.vector2i(), True)

    return fp

  def placePixel(self, reference, point, orientation, layer="F.Cu", allowOverlaps=True, alignOverlaps=True, powerConnect=None, groundConnect=None):
    print("Consider place pixel %s at point %s (relative center %s) orientation %f layer %s" % (reference, point, self.center, orientation, layer));
    from copy import deepcopy
    if powerConnect is None:
      powerConnect = deepcopy(self.pixelType.powerConnect)
    if groundConnect is None:
      groundConnect = deepcopy(self.pixelType.groundConnect)

    if self.pixel_prototype is None:
      print("Loading pixel footprint...")
      self.pixel_prototype = pcbnew.FootprintLoad(str(Path(__file__).parent.parent.joinpath('kicad_footprints.pretty').resolve()), self.pixelType.footprintName)
      self.pixel_prototype.SetLayer(self.kicadpcb.layertable[layer])

    absPoint = self.center + point

    if not allowOverlaps:
      overlapThresh = 0.8
      for fp in self.kicadpcb.board.GetFootprints():
        if not fp.GetReference().startswith(reference[0]):
          continue
        if not fp.GetLayerName() == layer:
          continue
        # print(fp, Point(fp.GetPosition()), fp.GetOrientation())
        fpPoint = Point(fp.GetPosition())
        fpOrientation = fp.GetOrientation().AsRadians()
        if absPoint.distance_to(fpPoint) < overlapThresh:
          if alignOverlaps:
            # the pixel overlaps with a pixel already on the board. we'll adjust it to 'flow' more with the new attemped pixel
            newPt = (fpPoint+absPoint)/2
            orientationTweakSign = 1 if abs(fpOrientation%(pi/2) - orientation%(pi/2)) > pi/4 else -1
            newOrientation = fpOrientation + (fpOrientation%(pi/2) - orientation%(pi/2)) / 2 * orientationTweakSign
            print("Pixel", reference, "overlaps", fp.GetReference(), "first", fpPoint, "@", fpOrientation, "second", absPoint, "@", orientation, " => ", newPt, "@", newOrientation)
            
            movedPads = [pad.GetPosition() for pad in fp.Pads()]
            
            fp.SetOrientation(pcbnew.EDA_ANGLE(newOrientation, pcbnew.RADIANS_T))
            fp.SetPosition(newPt.vector2i())

            # adjust any traces that were connected to this pixel
            for padNum in range(len(movedPads)):
              for track in self.kicadpcb.board.GetTracks():
                if Point(track.GetStart()).distance_to(movedPads[padNum]) < 0.1:
                  track.SetStart(fp.Pads()[padNum].GetPosition())
                elif Point(track.GetEnd()).distance_to(movedPads[padNum]) < 0.1:
                  track.SetEnd(fp.Pads()[padNum].GetPosition())
          else:
            print("Skipping pixel", reference, ", due to overlap")
          return None

    pixel = self.insertFootprint(self.pixel_prototype, reference, point, orientation, layer)

    # load orientation from the pixel, since flipping to the back side flips orientation too
    orientation = pixel.GetOrientation().AsRadians()
    # and flip it back anyway
    orientationFlip = pi if pixel.GetLayerName() == "B.Cu" else 0

    if not args.skip_traces:
      for pad in pixel.Pads():
        if groundConnect and int(pad.GetPadName()) == self.pixelType.groundPin:
          groundConnect.startPad = pad
          groundConnect.draw(self.kicadpcb)
        elif powerConnect and int(pad.GetPadName()) == self.pixelType.powerPin:
          powerConnect.startPad = pad
          powerConnect.draw(self.kicadpcb)

    return pixel

  def placeSeriesPixel(self, point, orientation, layer="F.Cu", allowOverlaps=True, alignOverlaps=True, series="D", reverseDataOrder=False, connectorFunction=None, powerConnect=None, groundConnect=None):
    reference = "{}{}".format(series, (self.seriesPixelCounts.get(series,0)+1))
    
    placedPixel = self.placePixel(reference, point, orientation, layer, allowOverlaps, alignOverlaps, powerConnect, groundConnect)
    if placedPixel is not None:
      if self.seriesPixelPrevious.get(series) is not None:
        fromPixel, toPixel = self.seriesPixelPrevious[series], placedPixel
        if reverseDataOrder:
          toPixel, fromPixel = fromPixel, toPixel
        self.connectPixels(fromPixel, toPixel, connectorFunction)

      self.seriesPixelPrevious[series] = placedPixel
      self.seriesPixelCounts[series] = self.seriesPixelCounts.get(series,0) + 1
    return placedPixel

  def seriesPixelDiscontinuity(self, series="D"):
    self.seriesPixelPrevious[series] = None

  def hideAllLabels(self):
    hid = 0
    for m in self.board.GetFootprints():
      if m.Reference().IsVisible():
        m.Reference().SetVisible(False)
        hid+=1
    print("Hid %i reference labels" % hid)

  def doLayout(self, args):
    needSave = False

    if args.dump_objects:
      self.kicadpcb.dumpObjects(args.dump_objects)
      print("Exiting after object dump")
      exit(0)

    if args.delete_all_traces:
      self.kicadpcb.deleteAllTraces()
      needSave = True

    if args.delete_all_drawings:
      self.kicadpcb.deleteAllDrawings()
      needSave = True
    
    if args.hide_pixel_labels:
      hid = 0
      for m in self.board.GetFootprints():
        if m.GetReference().startswith('D'):
          if m.Reference().IsVisible():
            m.Reference().SetVisible(False)
            hid+=1
      print("Hid %i reference labels" % hid)
      needSave = True
    if args.hide_all_labels:
      self.hideAllLabels()
      needSave = True

    if args.draw_poly:
      self.drawPoly(args.draw_poly)
      needSave = True
    if args.draw_wave:
      self.drawWave(args.draw_wave)
      needSave = True
    if args.draw_ellipse:
      self.drawEllipse(args.draw_ellipse)
      needSave = True
    if args.zone_circle:
      print(args.zone_circle)
      zc = args.zone_circle
      self.drawZoneCircle(Point(float(zc[0]), float(zc[1])), float(zc[2]), zc[3], zc[4], zc[5])
      needSave = True

    if args.do_layout:
      self.kicadpcb.deleteAllDrawings(layer='F.Silkscreen')
      
      self.deleteZones()

      self.kicadpcb.deleteEdgeCuts()
      self.drawEdgeCuts()

      self.deletePixels()
      self.deleteStrayVias()
      self.placePixels()
      
      self.decorateSilkScreen()
      needSave = True
    elif args.do_silk:
      self.kicadpcb.deleteAllDrawings(layer='F.Silkscreen')
      self.decorateSilkScreen()
      needSave = True

    if args.stats:
      self.kicadpcb.dumpStats()

    if not args.dry_run and needSave:
      self.kicadpcb.save()

  def deleteStrayVias(self):
    self.kicadpcb.deleteStrayVias()

  ### to override
  def drawEdgeCuts(self):
    pass
  
  def deletePixels(self):
    import re
    for fp in self.kicadpcb.board.GetFootprints():
      if re.match("^D\d+$",fp.GetReference()):
        print("Removing old footprint for pixel %s.." % fp.GetReference())
        self.kicadpcb.deleteFootprintConnectedItems(fp)
        print("    Delete footprint %s" % fp.GetReference())
        self.kicadpcb.board.Delete(fp)
        self.kicadpcb.resetCache()

  def deleteZones(self):
    zones = self.kicadpcb.board.Zones()
    print("There are {} zones to delete: ".format(len(zones)), [z.GetZoneName() for z in zones])
    for zone in zones[::-1]: # list changes live when we delete these and fast-iteration doesn't manage it
      print("Removing old zone", zone.GetZoneName(), zone)
      if zone.GetZoneName() is None or len(zone.GetZoneName())==0:
        print("WARNING: Zone {} has no name".format(zone))
      else:
        self.kicadpcb.board.Delete(zone)

  def placePixels(self):
    pass
  
  def decorateSilkScreen(self):
    pass


## End Library Code, Begin Per-Project Code #############################################################################

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
    capPrototype = pcbnew.FootprintLoad(str(Path(findFootprintLibsFolder()).joinpath("Capacitor_SMD.pretty")), "C_0402_1005Metric")

    def insertCap(pos, orientation):
      nonlocal capIndex
      reference = "C{}".format(capIndex)
      cap = self.insertFootprint(capPrototype, reference, pos, orientation, "F.Cu")
      capIndex+=1
      for i, pad in enumerate(cap.Pads()):
        TraceBuilder(pad).angleConst(0.4, -orientation + i*pi - pi).via().draw(self.kicadpcb)
      return cap

    while True:
      orientation = pi/4
      placed = None
      if self.is_point_in_hexa(pos):
        if pos.x < 0 and xDirection > 0 and not placedLeftCap:
          angle = -3*pi/4
          angleAdjust = 0 if pos.y < 0 else -pi/2
          cap = insertCap(pos.polar_translated(2.4,angle+angleAdjust), orientation+angleAdjust)
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
        powerConnect = TraceBuilder().angleConst(0.7256, pi/2).angleConst(0.2387, pi/4).via() if xDirection>0 else TraceBuilder().angleConst(0.8, pi/2).angleConst(0.35, 3*pi/4).via()
        placed = self.placeSeriesPixel(pos, orientation, allowOverlaps=False, alignOverlaps=False, 
                                       powerConnect=powerConnect,
                                       groundConnect=TraceBuilder().angleConst(gndDistance, -pi/2).via(),
                                       connectorFunction=dataTraceConnector)
      else:
        if pos.x > 0 and xDirection > 0 and not placedRightCap:
          # just fell off the right end of the hexa
          lastPos = pos - (xDirection*spacing.x, 0)
          angle = -pi/4
          angleAdjust = 0 if lastPos.y < 0 else pi/2
          insertCap(lastPos.polar_translated(2.4,angle+angleAdjust), orientation+angleAdjust+pi/2)
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

    # draw hexa-crisscrossy lines on silk
    def rampUpDown(value, A, B, N):
      midpoint = lineCount//2
      if value < midpoint:
        return A + (B - A) * (value / midpoint)
      else:
        return B - (B - A) * ((value - midpoint) / midpoint)

    spacing = sin(2*pi/6)*self.pixelSpacing
    lineCount = floor(self.edgeRadius*2/spacing/2)*2-1
    edgeClearance = 0.6
    for t in range(3):
      theta = 2*pi/6*t
      for i in range(lineCount):
        y = -lineCount/2*spacing + i*spacing + spacing/2
        length = rampUpDown(i, self.edgeRadius, 2*self.edgeRadius, lineCount) - edgeClearance/2
        P1 = Point(-length/2, y).rotate_about(theta, Point(0,0))
        P2 = Point(length/2, y).rotate_about(theta, Point(0,0))
        # checking if we're (almost) in hexa serves the somewhat futile purpose of avoiding clipping the silkscreen on the board edge
        if self.is_point_in_hexa(P1, self.edgeRadius-edgeClearance/4) and self.is_point_in_hexa(P2, self.edgeRadius-edgeClearance/4):
          self.drawSegment(P1, P2)

layout = LayoutHexa(args.path, SK9822EC20)    

if __name__ == '__main__':
  layout.doLayout(args)

