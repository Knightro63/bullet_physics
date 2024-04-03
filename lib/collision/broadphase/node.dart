import 'package:bullet_physics/collision/broadphase/dbvt_aabb_mm.dart';

////////////////////////////////////////////////////////////////////////////

class Node {
  final DbvtAabbMm volume = DbvtAabbMm();
  Node? parent;
  final List<Node?> childs = [null,null];
  Object? data;

  bool isleaf() {
    return childs[1] == null;
  }

  bool isinternal() {
    return !isleaf();
  }
}

/** Stack element */
class sStkNN {
  Node? a;
  Node? b;

  sStkNN(Node? na, Node? nb) {
    a = na;
    b = nb;
  }
}

class sStkNP {
  Node? node;
  int mask = 0;

  sStkNP(Node? n, [int m = 0]) {
    node = n;
    mask = m;
  }
}

class sStkNPS {
  Node? node;
  int mask = 0;
  double value = 0;

  sStkNPS([Node? n, int m = 0, double v = 0]) {
    node = n;
    mask = m;
    value = v;
  }
  
  void set(sStkNPS o) {
    node = o.node;
    mask = o.mask;
    value = o.value;
  }
}

class sStkCLN {
  Node node;
  Node parent;

  sStkCLN(this.node, this.parent);
}

class ICollide {
  void process(Node? n1, [Node? n2, double f = 0]) {}

  bool descent(Node? n) {
    return true;
  }

  bool allLeaves(Node? n) {
    return true;
  }
}

abstract class IWriter {
  void prepare(Node root, int numnodes);
  void writeNode(Node n, int index, int parent, int child0, int child1);
  void writeLeaf(Node n, int index, int parent);
}

class IClone {
  void cloneLeaf(Node n) {}
}