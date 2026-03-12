# 第2章 - Sketcher架构

## 2.1 Sketcher、求解器和接口

一个Sketcher对象（SketchObject类）包含了定义给定Sketch的所有几何图形和约束，以及可以应用于Sketcher以添加、删除或以其他方式修改此几何图形的高级函数。

然而，Sketcher对象依赖于专门的代码——一个名为PlaneGCS的几何约束系统，来执行约束的实际求解，这使得几何图形能够适应这些约束。

在此架构级别上，最重要的概念是理解PlaneGCS不理解Sketcher中使用的几何和约束概念。对于程序员来说，它不知道Part::Geometry和Sketcher::Constraint是什么。

尽管这最初可能看起来像是设计缺陷或限制，但事实并非如此。这些对现实的更高层次表示对于Sketcher的其余操作非常重要，但在讨论几何形状引入的自由度数量或在寻求收敛时计算梯度以决定元素移动方向时则毫无用处。同样，求解过程中使用的信息在更高层次上大多无用。当然，本可以决定将所有内容放在一起，但这不会有任何优势，而且会失去很多灵活性，例如，如果有朝一日创建了更好的求解器实现，能够切换到该实现，或在其他产品中使用PlaneGCS（是的，除了FreeCAD之外还有其他产品使用PlaneGCS）。

拥有理解几何表示的更高级别的Sketcher和使用不同表示的求解器，使得有必要在两者之间引入接口以连接两者。这个接口，我们将称之为求解器接口，就是Sketch类。

**图2.1：Sketcher/求解器基本架构**
```
SketchObject
   ↓
Geometry + Constraints → 求解器接口(Sketch) → 求解器(PlaneGCS)
```

因此，现在应该清楚，当提到求解器架构时，Sketcher有三个主要部分：

- **Sketcher**：SketchObject类
- **求解器接口**：Sketch类  
- **求解器**：PlaneGCS目录中的所有内容

后者是本文档的主要关注点。

## 2.2 Sketcher几何图形和约束

我们指的是在SketchObject级别使用的更高级别的几何图形和约束。

Sketcher中使用的几何图形在Geometry.h中定义，所有几何图形都派生自Part::Geometry。Sketcher有一个指向此类对象的指针向量，`std::vector<Part::Geometry *>`。

这些Geometry类是OpenCASCADE Geometry类的包装器，它们添加了保存几何图形时的存储能力以及简化某些常见任务的高级函数。

Sketcher中使用的约束在Constraint.h中定义（不要与planegcs目录中的Constraints.h混淆）。Sketcher有一个指向此类对象的指针向量，`std::vector<Sketcher::Constraint*>`。

这些约束仅定义同一或不同几何元素各部分之间的关系。每个约束都有一个类型，即枚举Sketcher::ConstraintType，例如Sketcher::Horizontal表示水平约束。

它们包含3个整数值，称为First、Second和Third。每个值都能够存储Geometry数组中Geometry对象的索引。有一个Constraint::GeoUndef值用于在未存储值时使用。例如，水平约束只需要存储一个几何索引，即应用该约束的线的索引，该索引将存储在First中。Second和Third将设置为GeoUndef。然而，对称约束在最一般情况下需要三个不同的几何索引。

除了这3个整数值外，约束还包含3个Sketcher::PointPos枚举：FirstPos、SecondPos、ThirdPos，它们可以取值Sketcher::none、Sketcher::start、Sketcher::end、Sketcher::mid。它们默认为Sketcher::none。使用这些值，可以指示约束应用于边（Sketcher::none）、起点（Sketcher::start）、终点（Sketcher::end）或中心（Sketcher::mid）。

当然，并非所有几何元素都具有所有可能的组合（圆没有起点或终点，线没有中心），但所有可能的组合定义了这个更高层Constraint可以指示的限制。

在将椭圆和椭圆弧引入Sketcher时，曾详细讨论过每个几何图形限制三个可寻址顶点的问题。增加可寻址顶点数量的问题是双重的：a）新元素可能需要可变数量的顶点（例如B样条），b）它将需要对Sketcher和Sketcher接口进行重大更改。最终，解决方案作为一种新型约束出现：对齐约束（Alignment constraint）。这种约束类型有一个子类型Sketcher::InternalAlignmentType，它包含特定于几何的子类型，例如Sketcher::EllipseFocus1，它标识第二个几何图形（在这种情况下是一个点）要对齐到椭圆的那个位置。值得注意的是，几何图形不限于点。它可以是线，例如椭圆的主轴，或圆，例如B样条的控制点。这在实现中提供了高度的灵活性，并支持复杂形状。然而，如后续部分所述，使用内部对齐约束获得的灵活性是以必须编写特定对齐代码为代价的。

## 2.3 求解器

### 2.3.1 参数

planeGCS求解器围绕参数概念构建。每个几何图形都有一定数量的定义它的参数。每个不受约束的参数贡献一个自由度（DoF）。例如，点可以在x和y方向上移动，因此它有2个自由度。线有两个点，每个点都可以在x和y方向上移动，因此它有4个自由度。

关于参数和自由度，有一个非常重要但不太明显的方面需要注意。说一个不受约束的参数贡献一个自由度，与说一个受约束的参数贡献零个自由度是不同的。如下文将更详细解释的，约束是一个数学上限制一个或多个参数值的单一方程。不受约束的参数可以在实数域中取任何值。受约束的参数只能取实数域的子集。

受约束参数的一个特殊情况是当参数只能取一个值时（其值是固定的）。特别重要的是要注意，受约束的参数仍然可以用不同的值满足所有约束。只有当Sketcher的所有参数都被相等数量的约束约束，且这些约束既不冗余也不冲突时，我们才能说Sketcher是完全受约束的。

让我们用一个例子来说明重要概念。假设一个系统有两个参数（p1和p2），例如一个只有一个点的Sketcher，并且只有一个这样的方程，可能出现三种不同情况：

a) p1固定为某个值而p2可以自由移动（设置水平距离约束，当拖动时点在垂直方向移动）
b) p2固定为某个值而p1可以自由移动（设置垂直距离约束，当拖动时点在水平方向移动）
c) 约束定义了p1和p2之间的关系，该关系受到约束（到原点的距离约束，当拖动时点围绕原点沿圆形轨迹移动）

所有这些情况都有1个自由度。在第一种情况下，只有参数p1受到约束，因此p2不受约束。在第二种情况下，只有参数p2受到约束，因此p1不受约束。在最后一种情况下，两个参数都受到约束，尽管约束数量不足以导致0个自由度。

在内部，参数不过是指向double的指针，double*。因此可以在实数域中取具有特定精度的值。

### 2.3.2 求解器几何图形

如前所述，planeGCS对几何图形的表示与Sketcher的表示大不相同。几何图形在Geo.h中定义。这些Geometry类的目的有很多，最重要的功能是：

1. 跟踪每个几何图形的参数
2. 存储特定于几何图形的求解器信息
3. 促进方程组的创建
4. 简化更复杂几何图形的定义

每个几何图形直接或间接继承自DependentParameters类。此类充当反馈与每个特定几何图形相关的求解器信息的接口。目前，该类只有一个成员hasDependentParameters，默认为false，并在求解过程中由求解器更新，从而能够识别给定几何图形是否有任何依赖参数。这里的"依赖"意味着求解器识别为未达到被约束到单一值的特殊状态的参数。

planeGCS求解器中最简单的几何图形类是Point，它毫不意外地存储两个分别指向double的指针，每个维度一个。这样的点不仅是Sketcher中单个点的表示，而且在求解器级别定义更复杂的几何图形（例如线段或椭圆）时被大量使用。

在进一步描述几何图形之前，有必要介绍一个求解器特定的向量概念，它已成为定义求解器约束的基本要素：DeriVector2。此类存储一个二维向量以及相对于给定参数的偏导数。当然，此类可以用作普通的二维向量，忽略导数，从而产生内存惩罚。然而，它擅长于同时无缝计算向量函数及其导数。支持通用向量运算，如规范化、标量积、求和、减法、缩放、旋转和线性组合。目前，理解DeriVector2是一种特殊类型的向量就足够了，它能够指示二维位置或向量，以及该向量相对于给定参数的导数（即当参数值变化时向量如何变化）。这个发明的优势将在整篇文档中对读者变得明显。

求解器级别最重要的几何图形类是抽象类Curve。除了重用代码而不是复制代码的一般原则外，它还对从它派生的任何其他曲线强制执行重要要求，即实现纯虚函数CalculateNormal：

```cpp
virtual DeriVector2 CalculateNormal(Point &p, double* derivparam = 0) = 0;
```

乍看之下，实现这样的函数可能看起来微不足道。然而，实现它避免了必须实现多个约束，这些约束将在通用例程中使用该函数的结果。

还有另外三个纯虚函数：

```cpp
virtual DeriVector2 Value(double u, double* derivparam = 0) = 0;
virtual double getRadMaj(Point &p, double* derivparam = 0) = 0;
virtual double getRadMin(Point &p, double* derivparam = 0) = 0;
```

其中，误差和梯度的计算是几何约束求解器概念的核心。编写约束通常不是一件简单的事情。约束的基础是识别满足约束时的误差的数学表达式。

例如，对于椭圆上的点，此表达式用于误差：

```
e = √(F2,y−Py)² + (F2,x−Px)² + √(F1,y−Py)² + (F1,x−Px)² − 2a
```

此误差函数基于的原理是：椭圆上一点(P)到其焦点(F1和F2)的距离之和等于两倍长轴长度(a)。因此，给定不在椭圆上的点P，将存在误差，点离椭圆曲线越远，误差越大。因此，误差是一个指示几何图形对于该约束收敛到正确解的程度的大小的量。

主要限制是误差函数本身不允许确定参数应向哪个方向移动以减少该误差。这就是引入约束梯度的原因。对于给定约束，构成梯度的偏导数数量与其定义中涉及的参数数量一样多。

对于上述椭圆上的点的情况，涉及的参数包括：点的两个坐标(Px和Py)、椭圆左焦点的坐标(F1,x和F1,y)、椭圆中心点的坐标(Cx和Cy)、椭圆短轴的长度(r)。值得注意的是，本可以使用其他参数来定义椭圆，但上述参数是最终实现的参数。这意味着仅对于点在线上约束就必须计算七个不同的偏导数。这只是其中之一，对应于椭圆短轴长度的那个，因其相对于其他的简洁性而被选中：

```
∂e/∂r = −2r / √(r² + (F1,x−Cx)² + (F1,y−Cy)²)
```

在对约束内部工作原理进行此探讨后，约束最重要的成员函数可能不再令人惊讶：

```cpp
virtual double error();
virtual double grad(double *);
virtual double maxStep(MAP_pD_D &dir, double lim = 1.0);
```

如果稳定性需要较低的收敛速度，则最后一个函数可用于限制收敛期间的步长，但很少实现。

由于手动计算微分繁琐、容易出错、耗时，并且导致不能立即被人类理解的非显而易见的公式，因此创建了DeriVector2。

引入DeriVector2后，没有使用如上所示的手动公式编写任何约束，并且一些已经编写且存在问题的约束已使用DeriVector2重写。使用DeriVector2，误差和梯度使用同一个函数计算，通常称为errorgrad，然后从error和grad函数中调用它，传递正确的参数。

ConstraintEllipseTangentLine::errorgrad是一个很好的示例。error函数基于这样一个性质：椭圆的一个焦点与关于椭圆切线镜像的另一个焦点的位置之间的距离具有给定长度2a。

```cpp
DeriVector2 p1(l.p1, param);
DeriVector2 p2(l.p2, param);
DeriVector2 f1(e.focus1, param);
DeriVector2 c(e.center, param);
DeriVector2 f2 = c.linCombi(2.0, f1, -1.0); // 2*cv - f1v

// 将F1关于直线镜像
DeriVector2 nl = l.CalculateNormal(l.p1, param).getNormalized();
double distF1L = 0, ddistF1L = 0; // F1到直线的距离
distF1L = f1.subtr(p1).scalarProd(nl, &ddistF1L);

// f1m = f1镜像后的点
DeriVector2 f1m = f1.sum(nl.multD(-2*distF1L, -2*ddistF1L));

// 计算f1m到f2的距离
double distF1mF2, ddistF1mF2;
distF1mF2 = f2.subtr(f1m).length(ddistF1mF2);

// 计算长轴半径（用于比较距离）
double dradmin = (param == e.radmin) ? 1.0 : 0.0;
double radmaj, dradmaj;
radmaj = e.getRadMaj(c, f1, *e.radmin, dradmin, dradmaj);

if (err)
    *err = distF1mF2 - 2*radmaj;
if (grad)
    *grad = ddistF1mF2 - 2*dradmaj;
```

梯度取决于求解器正在评估的参数，因此第一步是计算要与椭圆相切的直线的端点位置，以及这些位置如何随着被评估参数的变化而变化，即相对于参数的偏导数。类似地，计算左焦点、椭圆中心或右焦点如何随参数变化。虽然前者是直线和椭圆的参数，但后者，右焦点，在知道剩余参数时已定义，因此不是用于定义椭圆的参数之一。出于这个原因，它使用这两个点的线性组合从椭圆中心和左焦点计算得出。

接下来获得直线起点处的法向量，包括偏导数。然后将向量微积分应用于这些信息，以获得具有参数当前值的表达式的误差和梯度。梯度使求解器知道应如何修改参数以最小化误差。

值得注意的是，使用DeriVector2可以同时计算误差和梯度，并且如何实现最小化仍然可以被人类理解。

### 2.3.4 系统和子系统

求解器的核心是GCS.h中的GCS::System类。它包含所有参数和约束，特别是将参数解耦为组件。这些组件定义了可以独立求解的参数和约束的子集。这些组件就是子系统。

GCS中有两种类型的子系统：
- a) 源自无优先级约束的子系统
- b) 源自优先级约束的子系统Aux

优先级用于涉及增强解或几何图形程序化移动的某些操作。

SubSystem.h中的GCS::Subsystem类对子系统的参数和约束执行 several 计算操作，例如残差、雅可比或梯度的计算。这些计算被GCS::System中定义的求解器算法（例如DogLeg）使用。

## 2.4 求解器接口

考虑到Sketcher、SketchObject、几何图形和约束与求解器之间的巨大差异，存在一个处理两者之间双向交互的中介就不足为奇了。

虽然这里可以包含关于接口的更高程度的细节，但出于教学原因，它将推迟到介绍求解器支持的Sketcher功能之后再进行。

---

*本翻译文档基于FreeCAD Sketcher Solver Architecture.pdf, Abdullah Tahiri, December 2018*
