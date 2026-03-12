# 第3章 - Sketcher函数

## 3.1 引言

虽然在不考虑求解器向Sketcher提供的功能的情况下理解底层求解器算法是完全可能的，但事先介绍这些功能有助于更好地理解为求解器接口选择的架构以及求解器特性背后的原因。

本章描述的所有功能都在SketchObject.h的SketchObject中实现。

## 3.2 Sketch的修改

这是Sketcher最重要的功能，它使几何图形能够适应用户设置的一些Sketcher约束。可以思考在Sketcher的编辑模式下进行操作时的情形。

事实上，当Sketcher处于编辑模式时，对于引入的修改有两种求解可能：
- **a) 求解（solve）**
- **b) 重新计算（recompute）**

从用户界面的角度来看，这由求解器消息任务面板中的"自动更新"复选框控制。当勾选时，会触发重新计算，否则会触发求解。

主要区别在于，重新计算会将所有对Sketcher的更改传播到依赖于它的Sketcher外的任何对象，而求解则不会。因此，求解不会改变从编辑模式外部看到的Sketcher几何图形。当然，重新计算包含求解，但相关的是它们是独立的函数。

实现求解功能的C++函数是：

```cpp
int solve(bool updateGeoAfterSolving=true);
```

调用此函数涉及以下步骤：

**1. 重置任何正在进行的拖动或程序化几何图形移动过程：**
```cpp
solvedSketch.resetInitMove();
```

**2. 将几何图形、约束和外部几何图形的当前值传递给求解器接口，并请求对方程系统进行诊断。** 此评估会返回系统的自由度（DoF）以及是否存在冗余或冲突约束等信息。如果存在冗余或冲突约束，或者系统过约束，则求解过程停止。

```cpp
lastDoF = solvedSketch.setUpSketch(
    getCompleteGeometry(),
    Constraints.getValues(),
    getExternalGeometryCount());

lastHasConflict = solvedSketch.hasConflicts();
lastHasRedundancies = solvedSketch.hasRedundancies();
lastConflicting = solvedSketch.getConflicting();
lastRedundant = solvedSketch.getRedundant();
```

**a) 方程系统被求解，并检索状态。** 然而，如果求解器失败，过程继续。

```cpp
lastSolverStatus = solvedSketch.solve();
```

**b) 如果求解器未失败且参数updateGeoAfterSolving已设置，则根据求解结果更新Sketcher的几何图形。**

重新计算将在下一节中与触发重新计算的其他情况一起处理。

## 3.3 重新计算

当对象的属性发生变化时，对象可能已被更改，必须采取适当的措施。FreeCAD中使用的机制称为重新计算，重新计算时执行的函数是：

```cpp
App::DocumentObjectExecReturn *execute(void);
```

调用此函数涉及以下步骤：

**a) 外部几何图形可能已更改，因此编辑模式表示被更新。** 如果此类更新会导致任何指向外部几何图形的约束失败，则这些约束将被移除。
```cpp
rebuildExternalGeometry();
```

**b) 执行updateGeoAfterSolving设置的求解。** 任何错误，无论是冗余或冲突约束，还是其他过约束，或实际的求解器失败，都会通知用户。
```cpp
int err = this->solve(true);
```

**c) 如果没有错误，则使用求解器获得的结果更新Sketcher的二维形状。**
```cpp
Shape.setValue(solvedSketch.toShape());
```

Sketcher本身在编辑模式的自动更新模式下可能会触发重新计算。这将直接把Sketch中的所有更改传播到依赖于它的其他对象。

同样，如果Sketch依赖于表达式或外部几何图形，并且它们发生变化，也会生成Sketch的重新计算。

## 3.4 移动Sketch几何图形

有时即使在求解器不要求此类移动以满足约束的情况下，也需要移动Sketch几何图形。这种移动功能可能是用户交互的结果。一个例子是拖动几何图形的操作。当需要以编程方式移动给定几何图形或其定义的顶点之一时，也会使用移动功能。一个例子与在Sketcher中创建圆角有关。创建圆角时，有必要以编程方式将现有几何图形的端点（要应用圆角的几何图形）移动到定义圆角的圆弧的起点和终点。这必须在执行任何求解之前完成，否则求解器将自由移动圆弧和几何图形，导致不期望的行为。

虽然前两个功能直接或间接依赖于之前看到的Sketcher求解功能：
```cpp
int solve(bool updateGeoAfterSolving=true);
```

但顶点的程序化移动使用SketchObject函数：
```cpp
int movePoint(int GeoId, PointPos PosId,
              const Base::Vector3d& toPoint,
              bool relative=false,
              bool updateGeoBeforeMoving=false);
```

调用此函数：

**a) 如果根据参数updateGeoBeforeMoving在移动前需要更新几何图形，则更新求解器几何图形。** 否则，使用求解器级别可用的几何图形进行移动操作。

**b) 如果求解器的上次执行返回了过约束的Sketch或具有冲突约束的Sketch，则中止几何图形的移动。**

**c) 调用求解器接口的点移动功能。** 此功能内部执行一种特殊类型的求解，返回求解状态。
```cpp
lastSolverStatus = solvedSketch.movePoint(GeoId, PosId, toPoint, relative);
```

**d) 如果此特殊求解成功，则根据找到的解更新Sketcher的几何图形。**

**e) 当移动操作完成时，重置求解器接口移动功能。**
```cpp
solvedSketch.resetInitMove();
```

在用户交互的情况下，它使用类似的接口，因为即使它响应用户交互，最终也不过是根据用户输入进行程序化移动。然而，有两个主要有意义的差异：
- a) 它直接由ViewProvider接口
- b) 它使用求解器接口的附加方法

第一个差异是用户交互的明显结果。第二个差异是正在进行的移动操作需要中间更新的结果。在拖放的特殊情况下，为了在拖动过程中重绘并给出交互的印象。

此类动态操作的示例可以在ViewProviderSketch.h中找到：
```cpp
bool ViewProviderSketch::mouseMove(
    const SbVec2s &cursorPos,
    Gui::View3DInventorViewer *viewer)
```

详细讨论此函数不值得，但重要的是要识别此类操作以对求解器接口的以下函数的调用开始：
```cpp
getSketchObject()->getSolvedSketch().initMove(GeoId, PosId, false);
```

后续调用直接指向求解器接口：
```cpp
getSketchObject()->getSolvedSketch().movePoint(GeoId, PosId, vec, false)
```

然而，当拖放操作结束时，最终移动通过SketchObject生效，以使用求解器几何图形更新Sketcher几何图形。

## 3.5 Sketcher相关的求解器信息

求解器返回两种类型的信息：
- **a) 通用信息**
- **b) 特定信息**

第一种类型在每次触发求解器接口设置Sketch或求解时获得。第二种类型仅在完整求解操作完成后按需获得。

如3.2节已经介绍的，必须首先向求解器提供它必须求解的几何图形，这使用求解器接口的setUpSketch函数执行。此函数执行方程系统的初始评估或诊断。从此初始评估获得重要信息：
- a) 系统的自由度数量
- b) 是否存在冗余约束以及它们是哪些
- c) 是否存在冲突约束以及它们是哪些

此信息使Sketcher能够知道是否值得尝试实际求解系统。但它还告知用户存在的自由度数量以及必须处理哪些与约束相关的问题以实现有效的几何解。

```cpp
lastDoF = solvedSketch.setUpSketch(
    getCompleteGeometry(),
    Constraints.getValues(),
    getExternalGeometryCount());

lastHasConflict = solvedSketch.hasConflicts();
lastHasRedundancies = solvedSketch.hasRedundancies();
lastConflicting = solvedSketch.getConflicting();
lastRedundant = solvedSketch.getRedundant();
```

如果诊断显示没有问题，则执行实际求解并获得附加信息：
```cpp
lastSolverStatus = solvedSketch.solve();
```

此状态信息在GCS.h中定义，可能取多个值：
- **GCS::Success**: 解使误差函数为零
- **GCS::Converged**: 解最小化误差函数，但不为零
- **GCS::Failed**: 求解器未能找到解
- **GCS::SuccessfulSolutionInvalid**: 求解器找到了解，但此解不被OpenCASCADE接受

实际上，此信息目前在Sketcher级别仅以二进制形式使用，即GCS::Success或GCS::Failed。此信息在求解器消息任务面板中反馈给用户。

关于特定信息，目前求解器能够识别在2.3.2节和2.3.3节定义内具有依赖参数的几何图形。然而，这无法通过求解器默认用于稀疏矩阵QR分解的更快算法SparseQR实现。因此，为了实现它，需要使用全主元稠密QR分解请求特定求解。假设Obj是SketchObject的实例，可以通过手动设置QR算法来请求此特定求解：

```cpp
Obj->getSolvedSketch().setQRAlgorithm(GCS::EigenDenseQR);
Obj->solve(false);
Obj->getSolvedSketch().setQRAlgorithm(GCS::EigenSparseQR);
```

求解后，可以针对给定的几何索引geoId和给定的Sketcher::PointPos pos查询求解器接口，如下所示：
```cpp
Obj->getSolvedSketch().hasDependentParameters(geoId, pos);
```

在CommandSketcherTools.cpp中可以找到如何获取此信息的完整示例：
```cpp
void CmdSketcherSelectElementsWithDoFs::activated(int)
```

或者，SketchObject也有一个函数，返回包含所有依赖参数的geo索引和PointPos对的向量。这也从Python可访问。

```cpp
void SketchObject::getGeometryWithDependentParameters(
    std::vector<std::pair<int,PointPos>>& geometrymap)
```

---

*本翻译文档基于FreeCAD Sketcher Solver Architecture.pdf, Abdullah Tahiri, December 2018*
