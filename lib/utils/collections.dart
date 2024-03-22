import 'package:bullet_physics/utils/object_array_list.dart';

class Collections{
  static swap<T>(List<T> list, int from, int to){
    T s1 = list[from];
    T s2 = list[to];
    list.insert(to, s1);
    list.insert(from, s2);
  }

  static swapObjectArray<T>(ObjectArrayList<T?> list, int from, int to){
    T? s1 = list.array[from];
    T? s2 = list.array[to];
    list.array.insert(to, s1);
    list.array.insert(from, s2);
  }
}