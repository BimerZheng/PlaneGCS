# 第4章 - 求解器接口

## 4.1 引言

求解器接口最重要的功能是在Sketcher（SketchObject）和求解器（PlaneGCS）之间进行接口。这主要涉及在求解之前将几何图形和约束信息从其Sketcher形式转换为求解器可用的形式，并将嵌入在求解器几何图形中的结果传回Sketcher。

为了有效实现此功能，求解器接口必须跟踪Sketcher和求解器，为此它维护多个数据结构。

如第3章已经介绍的，求解器提供接口以实现：
- a) 求解
- b) 移动几何图形
- c) 检索求解器信息

## 4.2 数据结构

### 4.2.1 几何图形

求解器接口级别的几何图形结构化为求解器接口定义类型的向量GeoDef，以及包含每个几何图形的求解器表示的多个特定于几何类型的向量：

```cpp
std::vector<GeoDef> Geoms;
std::vector<GCS::Point> Points;
std::vector<GCS::Line> Lines;
std::vector<GCS::Arc> Arcs;
std::vector<GCS::Circle> Circles;
std::vector<GCS::Ellipse> Ellipses;
std::vector<GCS::ArcOfEllipse> ArcsOfEllipse;
std::vector<GCS::ArcOfHyperbola> ArcsOfHyperbola;
std::vector<GCS::ArcOfParabola> ArcsOfParabola;
std::vector<GCS::BSpline> BSplines;
```

求解器接口定义的类型是一个结构体，包括指向Sketcher几何图形副本的指针、求解器接口定义的几何类型、指示几何图形是否为外部几何图形的标志、对应于此几何图形在包含求解器表示的特定几何类型向量中的位置的索引，以及包含求解器表示的点的向量中形状的三个可能可寻址顶点中每个顶点的索引。

```cpp
Part::Geometry * geo;
GeoType type;
bool external;
int index;
int startPointId;
int midPointId;
int endPointId;
```

通过这些信息，Sketcher跟踪几何图形并实现几何图形信息的双向同步。

### 4.2.2 约束

求解器接口级别的约束结构化为求解器接口定义类型的向量ConstrDef：
```cpp
std::vector<ConstrDef> Constrs;
```

求解器接口定义的类型包括指向Sketcher约束的指针、指示约束是驱动约束还是参考约束的标志、指向约束数据值内存地址的第一个double指针（如果它有一个），以及指向约束第二个数据值内存地址的第二个double指针（如果它有一个）。后者仅在Snell定律约束的情况下使用。

```cpp
Constraint * constr;
bool driving;
double * value;
double * secondvalue;
```

### 4.2.3 参数

如2.3.1节所介绍的，求解器基于参数工作。当在求解器接口处创建求解器几何图形时，Sketcher接口的另一个重要功能是为这些参数分配内存并跟踪它们。

有一个指向double的第一个向量，包含系统中所有未固定的参数（例如外部几何图形的参数），第二个向量仅包含那些被驱动的参数（即参考约束的数据），第三个向量仅包含固定的参数（例如外部几何图形的参数），第四个向量包含在使用全主元稠密QR分解进行诊断后被检测为依赖参数的那些参数。然后还有两个额外的double数组（不是指向double的指针），它们存储用于以编程方式移动几何图形的参数。

```cpp
std::vector<double *> Parameters;
std::vector<double *> DrivenParameters;
std::vector<double *> FixParameters;
std::vector<double *> pconstraintplistOut;
std::vector<double> MoveParameters, InitParameters;
```

## 4.3 从Sketcher到求解器的数据传递

如前几节所述，使用求解器接口函数将Sketcher的几何图形传递给求解器：

```cpp
int setUpSketch(
    const std::vector<Part::Geometry *> &GeoList,
    const std::vector<Constraint *> &ConstraintList,
    int extGeoCount=0);
```

Sketcher几何图形是完整几何图形。这意味着它包含内部几何图形和外部几何图形在同一向量中。内部几何图形首先被推入向量，然后是外部几何图形。最后一个参数指示向量中存在多少外部几何图形。这允许计算内部和外部几何图形之间的边界。

由于这是将几何图形和约束添加到求解器的点，因此它也是决定向求解器发送哪些几何图形和约束的点。换句话说，如果由于某种原因某个几何图形或约束不应参与求解过程，或应以特殊方式参与，则需要在此点相应地进行处理。一个这样的例子是Block约束的实现，它将阻塞几何图形的参数设置为如同它们是外部几何图形一样（即固定的），并因此排除一些其他约束。

添加几何图形的函数遍历几何图形向量，考虑任何被阻塞的几何图形，检查Sketcher类型并执行特定于类型的添加函数。说明此过程的简单示例对应于函数：

```cpp
int Sketch::addLineSegment(
    const Part::GeomLineSegment &lineSegment, bool fixed)
```

以下是线段特定函数的示例。首先，根据几何图形是外部还是内部，选择正确的参数向量。其次，创建几何图形的副本。创建求解器接口定义类型GeoDef的对象，并分配指针和类型数据。创建点和线的相应求解器几何类型，并在相关参数向量中为起点和终点坐标分配内存。点被添加到点向量，线被添加到线向量。点在点数组中的索引被添加到GeoDef对象，该对象被添加到包含所有几何图形的Sketcher接口向量。

```cpp
std::vector<double *>& params = fixed ? FixParameters : Parameters;
GeomLineSegment *lineSeg = static_cast<GeomLineSegment*>(lineSegment.clone());
GeoDef def;
def.geo = lineSeg;
def.type = Line;
Base::Vector3d start = lineSeg->getStartPoint();
Base::Vector3d end = lineSeg->getEndPoint();
GCS::Point p1, p2;
params.push_back(new double(start.x));
params.push_back(new double(start.y));
p1.x = params[params.size()-2];
p1.y = params[params.size()-1];
params.push_back(new double(end.x));
params.push_back(new double(end.y));
p2.x = params[params.size()-2];
p2.y = params[params.size()-1];
def.startPointId = Points.size();
def.endPointId = Points.size()+1;
Points.push_back(p1);
Points.push_back(p2);
GCS::Line l;
l.p1 = p1;
l.p2 = p2;
def.index = Lines.size();
Lines.push_back(l);
Geoms.push_back(def);
return Geoms.size()-1;
```

添加约束的函数遍历约束向量，考虑任何不可执行的约束。对于每个Sketcher约束，它创建求解器接口定义类型ConstrDef的对象，分配约束指针并相应设置驱动标志。然后检查约束类型。如果约束包含数据，为参数分配内存并分配约束的值，如果约束是驱动的，则将参数分配给FixParameters向量，如果是参考的，则分配给Parameters和DrivenParameters向量。这允许在它是驱动的情况下强制执行约束值，在它是参考的情况下允许求解器修改它。无论是否包含数据，都会调用特定于约束的函数来创建一个或多个相应的求解器约束。

此类函数的示例是：
```cpp
int Sketch::addDistanceXConstraint(int geoId, double * value, bool driving)
```

在此特定情况下，通过求解器差分约束实现此水平距离约束。首先检查约束的几何索引是否有效并对应于正确的几何类型。使用存储在GeoDef向量中的索引从线向量中检索线。创建标签。标签存储在求解器约束中。如果需要多个求解器约束来实现Sketcher约束，则所有求解器约束都携带相同的标签。这允许将求解器对其约束的发现映射到Sketcher约束。

```cpp
geoId = checkGeoId(geoId);
if (Geoms[geoId].type != Line)
    return -1;
GCS::Line &l = Lines[Geoms[geoId].index];
int tag = ++ConstraintsCounter;
GCSsys.addConstraintDifference(l.p1.x, l.p2.x, value, tag, driving);
return ConstraintsCounter;
```

将几何图形和约束添加到求解器后，求解器接口执行初始化和诊断：
```cpp
GCSsys.clearByTag(-1);
GCSsys.declareUnknowns(Parameters);
GCSsys.declareDrivenParams(DrivenParameters);
GCSsys.initSolution(defaultSolverRedundant);
```

这些求解器命令的实际功能将在专门介绍PlaneGCS的章节中解释。

## 4.4 求解

求解器接口的求解函数直接从Sketcher执行。然而，它也由实现几何图形程序化移动的求解器接口函数执行。该函数在sketch.cpp中定义：
```cpp
int Sketch::solve(void)
```

求解器包含多种算法：
- a) DogLeg
- b) BFGS
- c) Levenberg-Marquardt
- d) SQP

从这些算法中执行首选算法。如果成功，则更新Sketcher的几何图形。如果不成功，则依次触发其余算法，直到一个成功或全部失败。

## 4.5 从求解器到Sketcher的数据传递

求解成功后，应用求解器的几何图形解，并使用解更新Sketcher的几何图形。如果更新成功，则需要将非驱动约束的值更新为求解器找到的值。解可能无效，因为OpenCASCADE可能不接受它。在这种情况下，求解和几何图形将被回滚。

```cpp
if (ret == GCS::Success) {
    GCSsys.applySolution();
    valid_solution = updateGeometry();
    if (!valid_solution) {
        GCSsys.undoSolution();
        updateGeometry();
    }
    else {
        updateNonDrivingConstraints();
    }
}
```

几何图形的更新通过迭代求解器接口的GeoDef向量并使用索引检索参数并将其分配给GeoDef结构中的Sketcher几何图形副本来完成。几何图形分配在SketchObject级别执行，使用extractGeometry函数获取Sketcher几何图形的副本。这种复制和分配的原因是双重的：
- a) 使Sketcher几何图形的完整性独立于求解器可能无法达到解的潜在情况
- b) 必须设置SketchObject中的属性，以便更改触发属性，并且撤销函数中使用的属性历史得到正确维护

## 4.6 几何图形的程序化移动

如3.4节所述，处理几何图形程序化移动的求解器接口函数是：

```cpp
int Sketch::initMove(int geoId, PointPos pos, bool fine)
void Sketch::resetInitMove()
int Sketch::movePoint(int geoId, PointPos pos, Base::Vector3d toPoint, bool relative)
```

initMove函数通过geo索引geoId和PointPos枚举标识要移动的几何图形部分。该函数创建新的临时求解器约束，在寻找解时给予优先级。这些约束的目的是强制顶点或边内的点跟随通过movePoint给出的位置或向量。

以下简单示例说明了initMove函数的作用：

```cpp
if (Geoms[geoId].type == Point) {
    if (pos == start) {
        GCS::Point &point = Points[Geoms[geoId].startPointId];
        GCS::Point p0;
        MoveParameters.resize(2);
        p0.x = &MoveParameters[0];
        p0.y = &MoveParameters[1];
        *p0.x = *point.x;
        *p0.y = *point.y;
        GCSsys.addConstraintP2PCoincident(p0, point, -1);
    }
}
```

点几何图形可能只有一个部分，即PointPos Sketcher::start。从求解器几何图形中检索具有其参数的点。创建新点p0，参数指向MoveParameters向量。这些参数的值被初始化为与要移动的点的值相同。然后创建新的求解器约束，以指示求解器这两个点应在同一位置。值得注意的是，约束以-1的负标签值添加。这表示此约束优先于任何其他约束。如将解释的，求解器使用特殊算法求解具有优先级约束的系统。

initMove函数以以下语句结束：
```cpp
InitParameters = MoveParameters;
GCSsys.initSolution();
isInitMove = true;
```

当介绍求解器initSolution函数时，它们的相关性将更加明显。此时，注意到它初始化方程系统以使求解器算法能够对它们起作用就足够了。

实际的几何图形移动由movePoint函数实现。这里区分两种情况：单个程序化移动几何图形和交互式移动。对于单个程序化移动，无需调用上面的initMove函数，因为movePoint检查点是否已初始化。如果未初始化，它调用initMove函数来初始化它。

然而，在交互式移动的情况下，不希望每次移动调用都初始化解，因为这会导致不期望的结果，例如滞后。在这种情况下，当从初始点的移动距离超过初始移动的20倍时（即第一次调用movePoint时指示的距离），单次求解初始化才会更新。此机制防止拖动操作期间的跳跃。

有两种类型的移动：
- a) **相对移动**：参数toPoint指示从原始位置的移动向量，主要用于交互式移动，如拖动
- b) **绝对移动**：参数toPoint提供初始点应移动到的位置

内部工作原理在以下示例中说明：

```cpp
if (relative) {
    for (int i = 0; i < int(MoveParameters.size()-1); i += 2) {
        MoveParameters[i] = InitParameters[i] + toPoint.x;
        MoveParameters[i+1] = InitParameters[i+1] + toPoint.y;
    }
}
else if (Geoms[geoId].type == Point) {
    if (pos == start) {
        MoveParameters[0] = toPoint.x;
        MoveParameters[1] = toPoint.y;
    }
}
```

MoveParameters向量使用点应移动的新位置进行更新，以相对或绝对方式。

预期地，函数以求解结束，以强制执行创建到新位置的约束。

求解器接口无法知道交互式移动操作何时完成。因此，当操作结束时，在Sketcher级别调用resetInitMove函数。

## 4.7 检索求解器信息

大多数求解器信息直接从求解器本身获得，求解器接口仅充当缓冲区。冗余约束、冲突约束、自由度就是这种情况。缓冲区相关之处在于可以执行多个求解器级别的求解，并且仅在信息与Sketch状态相关时才更新此信息。例如，如果人工添加约束用于某些操作，则该求解器信息对用户毫无价值。

大多数诊断信息从setUpSketch函数中触发的求解器诊断获得。冗余约束、冲突约束、自由度和依赖参数就是这种情况。

```cpp
int setUpSketch(
    const std::vector<Part::Geometry *> &GeoList,
    const std::vector<Constraint *> &ConstraintList,
    int extGeoCount=0);
```

关于在2.3.2节和2.3.3节定义内具有依赖参数的几何图形，求解器接口执行处理，以将求解器可用的纯参数信息转换为Sketcher几何图形信息。

```cpp
bool hasDependentParameters(int geoId, PointPos pos) const
void calculateDependentParametersElements(void)
```

calculateDependentParametersElements函数迭代求解器接口级别的几何图形结构，std::vector<GeoDef> Geoms。对于每个几何图形，它检查求解器识别为依赖的参数是否属于该几何图形，并相应地设置hasDependentParameters标志。这将求解器中识别的单独参数映射到包含它们的求解器接口级别的几何元素。

除此之外，它还进一步识别每个几何图形通过PointPos可寻址的顶点，并根据顶点的参数将顶点设置为依赖。这使得能够分别获得关于几何元素和顶点的信息。

hasDependentParameters函数依赖此计算将这些求解器接口级别的几何图形映射到实际的Sketcher几何图形。当Sketcher几何图形由GeoId索引和PointPos识别时，该函数返回true或false。

---

*本翻译文档基于FreeCAD Sketcher Solver Architecture.pdf, Abdullah Tahiri, December 2018*
