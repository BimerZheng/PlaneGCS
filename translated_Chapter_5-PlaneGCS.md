# 第5章 - PlaneGCS求解器

## 5.1 引言

Sketcher与求解器接口交互，求解器接口再与求解器交互。求解器保存所有求解器约束以及关于将它们划分为子系统和其他求解策略的信息，并负责应用这些策略以获得有效解。

## 5.2 数据结构

### 5.2.1 引言

从数据成员的角度来看，在SketchObject.h中定义的SketchObject有一个求解器接口类型的数据成员，即在sketch.h中定义的sketch。反过来，求解器接口类型sketch有一个在GCS.h中定义的求解器类型GCS::System的数据成员。

求解器最重要的数据结构如下所示。当处理约束时，求解器约束存储在指向这些约束的指针向量中。将新约束推入约束向量时，创建两个邻接表，以便给定指向约束的指针可以获得参数，或给定参数获得涉及的约束。

```cpp
std::vector<Constraint *> clist;
std::map<Constraint *, VEC_pD> c2p;
std::map<double *, std::vector<Constraint *>> p2c;
```

向量plist指向求解器的参数列表。向量pdrivenlist指向求解器参数的子集，这些参数是参考约束的数据。它们通过求解器接口使用declareDrivenParams函数设置。

处理参数时，某些操作利用参数映射pIndex，其中参数作为键，映射返回给定参数在参数向量中的索引。此映射由declareUnknowns函数生成。

在进行任何求解操作之前，求解器使用setReferences函数将参数值存储在双精度引用向量中。这允许使用resetToReference函数将参数恢复到参考值。

```cpp
VEC_pD plist;
VEC_pD pdrivenlist;
MAP_pD_I pIndex;
VEC_D reference;
```

### 5.2.2 约束结构

约束是求解器的基础。在2.3.3节中介绍了求解器约束的核心求解功能。然而，这里将介绍关于约束通用结构的几个方面。

所有求解器约束都派生自Constraint类，其定义如下：

```cpp
class Constraint
{
protected:
    VEC_pD origpvec;
    VEC_pD pvec;
    double scale;
    int tag;
    bool pvecChangedFlag;
    bool driving;
public:
    Constraint();
    virtual ~Constraint() {}
    inline VEC_pD params() { return pvec; }
    void redirectParams(MAP_pD_pD redirectionmap);
    void revertParams();
    void setTag(int tagId) { tag = tagId; }
    int getTag() { return tag; }
    void setDriving(bool isdriving) { driving = isdriving; }
    bool isDriving() const { return driving; }
    virtual ConstraintType getTypeId();
    virtual void rescale(double coef = 1.0);
    virtual double error();
    virtual double grad(double *);
    virtual double maxStep(MAP_pD_D &dir, double lim = 1.0);
    int findParamInPvec(double* param);
};
```

约束存储约束的第一组参数，包含参数的原始值origpvec和第二组参数，包含当前值pvec。第一组仅用作恢复或将第二组中的当前值重定向的参考。

约束还有一个scale，用于缩放误差和梯度。可以使用rescale方法修改此scale。

tag是用于将求解器约束映射到Sketcher级别约束的索引。此外，负值用于优先级约束，这些约束行为特殊，在方程系统诊断期间不考虑。此类优先级约束用于例如几何图形的程序化移动。可以通过setTag和getTag函数进行接口。

pvecChangedFlag标志指示pvec向量已更改，类中保存的任何几何指针都应重建。

driving标志指示约束是被驱动还是参考。可以通过setDriving和isDriving函数进行接口。

params函数返回包含所有约束参数的指向double的向量。

约束有一个返回其类型的函数，类型是enum GCS::ConstraintType的值之一。

findParamInPvec函数在约束的参数中搜索给定参数，并返回第一次出现的索引，如果未找到则返回-1。

revertParams函数将参数向量重置为原始参数。

redirectParams函数根据映射更改约束的参数。基本上，它在映射中搜索约束的参数作为键，如果存在匹配，则将参数分配给映射中的关联参数。

约束实现的实际示例重用父类的基本功能，仅定义几个参数：

```cpp
class ConstraintDifference : public Constraint
{
private:
    inline double* param1() { return pvec[0]; }
    inline double* param2() { return pvec[1]; }
    inline double* difference() { return pvec[2]; }
public:
    ConstraintDifference(double *p1, double *p2, double *d);
    virtual ConstraintType getTypeId();
    virtual void rescale(double coef = 1.0);
    virtual double error();
    virtual double grad(double *);
};
```

约束使用三个参数p1、p2和d。其目标是保持这两个参数之间的差值。

构造函数基本上将参数分配给pvec并将origpvec初始化为相同的参数。上面的成员函数只是参数的命名访问器。

鉴于约束的简单性，查看误差和梯度函数很有意义：

```cpp
double ConstraintDifference::error()
{
    return scale * (*param2() - *param1() - *difference());
}

double ConstraintDifference::grad(double *param)
{
    double deriv = 0.0;
    if (param == param1()) deriv += -1;
    if (param == param2()) deriv += 1;
    if (param == difference()) deriv += -1;
    return scale * deriv;
}
```

## 5.3 从求解器接口到求解器的数据传递

### 5.3.1 引言

4.3节已经介绍了如何将几何图形和约束从Sketcher传递到求解器接口。虽然几何图形在求解器接口级别完全分配到参数中，但约束的传递依赖于求解器本身的特定函数。

### 5.3.2 约束创建

在上面介绍的示例中，负责将给定约束添加到求解器的函数实现如下：

```cpp
geoId = checkGeoId(geoId);
if (Geoms[geoId].type != Line)
    return -1;
GCS::Line &l = Lines[Geoms[geoId].index];
int tag = ++ConstraintsCounter;
GCSsys.addConstraintDifference(l.p1.x, l.p2.x, value, tag, driving);
return ConstraintsCounter;
```

在此示例中，addConstraintDifference函数是最终实现相应求解器约束创建的求解器函数示例：

```cpp
int System::addConstraintDifference(double *param1, double *param2, 
                                    double *difference, int tagId, bool driving)
```

此函数为求解器约束分配动态内存，并将构造函数中给定的参数添加到约束的pvec中。然后存储标签和驱动信息，并依赖通用求解器函数addConstraint将约束添加到系统。

```cpp
Constraint *constr = new ConstraintDifference(param1, param2, difference);
constr->setTag(tagId);
constr->setDriving(driving);
return addConstraint(constr);
```

### 5.3.3 约束添加

此通用求解器函数重置先前的诊断结果（如果添加了新约束，除非它具有负标签）。如前所述，标签是将求解器约束映射到Sketcher约束的机制。然而，除了该功能外，负标签还用于标记与Sketcher约束无关的求解器约束，而是人工添加以实现特定功能。一个例子是几何图形的程序化移动功能。这些临时添加的约束不会影响方程系统的诊断信息，这些信息对用户或求解过程是有意义的。

该函数将新约束推入求解器约束向量，并将约束中涉及的参数以及约束添加到参数和约束的邻接列表中。

```cpp
isInit = false;
if (constr->getTag() >= 0)
    hasDiagnosis = false;
clist.push_back(constr);
VEC_pD constr_params = constr->params();
for (VEC_pD::const_iterator param = constr_params.begin();
     param != constr_params.end(); ++param) {
    c2p[constr].push_back(*param);
    p2c[*param].push_back(constr);
}
return clist.size() - 1;
```

## 5.4 初始化解和诊断

返回4.3节中的Sketcher接口函数setUpSketch，在将几何图形和约束传递到求解器后有一些代码行，在4.3节中未解释。

```cpp
GCSsys.clearByTag(-1);
GCSsys.declareUnknowns(Parameters);
GCSsys.declareDrivenParams(DrivenParameters);
GCSsys.initSolution(defaultSolverRedundant);
```

第一行代码清除具有优先级标签-1的任何约束。在任何最终删除后，代码确保更新约束到参数的邻接列表。这有效地从系统中移除可能已添加的用于程序化移动几何图形的任何约束。

declareUnknowns函数初始化pIndex映射。

declareDrivenParams函数设置是参考约束数据的参数子集。

initSolution函数相当复杂，执行以下操作：
- 将当前参数值存储在参考向量中
- 使用QR分解对整个方程系统进行诊断，从而识别自由度、冗余和冲突约束，以及可选的依赖参数
- 识别任何解耦子系统并将原始系统划分为相应组件
- 识别标签ID >= 0的等式约束并准备相应的系统缩减
- 将剩余约束组织到两个子系统中，分别对应标签ID >= 0和< 0

### 5.4.1 诊断

诊断求解器的方程系统是一个相当复杂的过程。实现诊断的函数是：

```cpp
int System::diagnose(Algorithm alg)
```

诊断的目标是检测冗余和冲突约束，并计算方程系统的自由度。可选地，还可以识别未完全约束的参数。

如前几节所述，方程系统的诊断使用系统的雅可比矩阵执行，雅可比矩阵不过是具有每个约束相对于每个参数的梯度的矩阵。知道大多数约束仅依赖于有限数量的参数，毫不奇怪，这样的矩阵基本上是稀疏的（它有很多零元素）。这就是为什么支持稀疏算法很有趣，因为从计算角度来看它们要高效得多。

雅可比矩阵通常是非方阵矩阵（其维度为约束数量乘以参数数量），被分解为QR形式。QR分解是揭示秩的分解，意味着它能够获得秩，从而获得方程系统的自由度。一般而言，雅可比矩阵J的QR分解意味着找到一个正交矩阵Q，乘以梯形矩阵R得到雅可比矩阵J，因此J = QR。梯形矩阵R由上部三角矩阵和底部的零形成。

有不同的算法来获得QR分解。不同的算法和此类分解的变体。一些具有主元或全主元。一些是稠密算法，即为稠密矩阵设计的算法，而另一些是为稀疏矩阵设计的稀疏算法。毫不奇怪，不同的算法提供不同数量的附加信息。

一般而言，QR分解涉及转置和置换的矩阵运算。这些运算导致Q和R矩阵的行和列不对应于J的行和列，即使它们的乘积成功产生J。由于矩阵类型，此信息对于从梯形矩阵R提取附加信息特别重要，如冗余或冲突的实际约束，或未完全约束的参数，即系统的依赖参数。

用于QR分解的稠密算法是Eigen的FullPivHouseholderQR。此分解是类型PJP' = QR，其中P和P'是置换矩阵。此算法能够获取关于系统依赖参数的信息。

用于QR分解的稀疏算法是Eigen的SparseQR。此分解是类型JP' = QR，其中P'是列置换，它是填充减少和揭示秩的置换的乘积。

诊断中的一个历史问题来源是参考约束，在代码中称为驱动约束。驱动约束按定义创建依赖行，因为其数据值未被强制执行，而当与没有此类约束的情况相比时，它们人为地扩大了参数和约束的数量。这些问题通过从诊断中移除驱动约束来解决。因此，诊断不直接在参数向量plist上操作，而是在缩减版本pdiagnoselist上操作。

移除参考约束的缺点是很难按索引识别被检测为冗余或冲突的求解器约束。为了克服此限制，创建了映射jacobian-constraintmap，它为每个键分配在plist中的索引，该键具有在pdiagnoselist中相应索引。

当检测到冗余或冲突约束时，了解这些冗余或冲突求解器约束对应于单个Sketcher级别约束，或相应的Sketcher级别约束产生了多个求解器约束是相关的。因此，诊断函数使用tagmultiplicity映射，该映射将求解器约束的数量映射到键，该键是约束的标签。

雅可比矩阵通过为每个约束和每个参数计算约束相对于参数的梯度来创建：

```cpp
J(jacobianconstraintcount-1, j) = (*constr)->grad(pdiagnoselist[j]);
```

创建雅可比矩阵后，使用Eigen计算QR分解。由于与不同算法中置换信息的可用性相关的原因，实际分解的是转置雅可比矩阵。唯一的区别是参数在分解中显示为行，而约束显示为列。优点是列置换信息可用于所有使用的算法。

```cpp
qrJT.compute(J.topRows(jacobianconstraintcount).transpose());
paramsNum = qrJT.rows();
constrNum = qrJT.cols();
qrJT.setThreshold(qrpivotThreshold);
rank = qrJT.rank();
```

从实际角度来看，值得指出的是，为了确定秩，算法使用阈值，该阈值过滤掉虽然是可忽略的但不是零的数字。此可配置阈值通常设置在10^-13左右。另一个特殊性是代码中的R矩阵是梯形矩阵的上三角矩阵。

使用稠密算法时，Eigen提供行的转置矩阵。行对应于参数，因为QR分解在转置矩阵上执行。使用转置矩阵可以计算行的相应置换矩阵，该矩阵可用于将对应于R矩阵行的参数映射到J矩阵的参数。

R矩阵的前秩行对应于在转置雅可比矩阵中形成线性无关列的完全约束参数。剩余参数是未完全约束或依赖的参数，因此不贡献于系统的秩。这些依赖参数存储在pdependentparameters中。

不幸的是，使用SparseQR算法时，此参数检测似乎不起作用。很可能这是不可能的，或者仅仅是我们中没有一个人找到正确的解决方案。

诊断算法继续使用冲突或冗余约束的检测。如果约束数量超过系统的秩，则必然存在冗余或冲突约束。

考虑到梯形R矩阵，并从代数的角度考虑，考虑到我们的QR分解是在具有对应于系统N个参数的N行和对应于M个约束的M列的转置雅可比矩阵上执行的，Q矩阵也必须有N行，R矩阵也必须有M列。虽然与讨论无关，但值得注意的是，Q矩阵的列数和R矩阵的行数是相同的值（否则Q不能右乘以R），并且该值取决于系统的秩。

R矩阵有对应于上三角矩阵的第一行数和为零的第二行数。此上三角矩阵对角线上的值将在下文中称为主元。根据定义，在上三角矩阵中，主元下的所有元素（即对角线下）都为零。然而，对角线上方的值可以取任何值。

查看第一列，它只有一个非零值，第一个。如果考虑Q矩阵将如何右乘以R矩阵以产生转置雅可比矩阵，其第一列的值将仅基于此非零值确定。可以对三角矩阵执行零化简操作，通过应用转置操作。此零化简将显示一些独立参数正由多个约束定义，上三角矩阵R中零化简后出现的非零值指示哪些约束。这意味着这些约束要么是冲突的，要么是冗余的。可以使用Eigen提供的列置换矩阵将R的列映射到它们起源的原始约束。这样就有可能确定哪些求解器约束是冗余的。由于零化简后实际上可能出现多个非零值，对于移除的单个自由度，可能有多个求解器约束冲突或冗余。因此，识别约束组。在代码中，这组组称为：

```cpp
std::vector<std::vector<Constraint *>> conflictGroups(constrNum - rank);
```

鉴于此时不知道它们是冗余还是冲突，此名称可能看起来具有误导性。为了检测约束是冗余还是冲突，移除约束并尝试求解系统，如果尽管约束未参与求解过程但约束的误差很小，则约束是冗余的，而如果误差很大，则它们是冲突的。此求解在用户界面中称为冗余求解，并使用与正常求解相同的算法，尽管有差异。这些差异将在本文档后面考虑。此时注意到求解算法采用的参数之一仅指示冗余求解就足够了。

在求解之前还有一个主要问题。前述算法返回组内冲突或冗余的约束组。然而，当两个约束冗余时，必须决定移除它们中的哪一个。此外，同一求解器约束可能影响多个组。为了解决此问题，在这些组上执行启发式算法，直到已识别足够约束以供移除。这根据以下优先级规则识别要移除的约束：

- 更流行的约束，即在组中出现最多的约束
- 当流行度相同时，具有较低多重性的标签的约束，即与生成较少求解器约束数量的Sketcher约束相关联的约束
- 当流行度和多重性相同时，具有较高标签编号的约束，即在Sketcher级别引入最晚的约束

此启发式算法背后的理由是，如果存在多个问题，介入这些问题最多的约束是罪魁祸首。如果有多个这样的约束在这些问题中介入相同次数，最好移除与生成较少求解器约束数量的Sketcher约束相关联的求解器约束。原则是应消除与较不复杂Sketcher约束相关联的求解器约束，因为Sketcher约束可能部分地与另一个Sketcher约束冗余，即它可能生成多个求解器约束，其中此类求解器约束之一与从另一个Sketcher约束生成的求解器约束冗余，而其他则不是。

最后，如果流行度和多重性相同，则选择最近引入的约束。此标准是最不相关的，但由于无论如何都必须选择一个，它假设用户将有更好的机会适应那些冗余约束中最近引入的约束的预期移除。

为了确定选择移除的约束是冗余还是冲突，构建没有它们的方程系统并找到解。如果此类解存在，则检查每个约束的误差，并且那些误差在冗余求解的收敛阈值下的约束被标记为冗余求解器约束。

```cpp
SubSystem *subSysTmp = new SubSystem(clistTmp, pdiagnoselist);
int res = solve(subSysTmp, true, alg, true);
```

从编程角度来看，值得指出的是，求解后应用解，获得误差，并在确定约束是冗余还是冲突后，参数重置为参考值，以便此求解不影响将在以下部分中描述的解。

```cpp
subSysTmp->applySolution();
for (std::set<Constraint *>::const_iterator constr = skipped.begin();
     constr != skipped.end(); ++constr) {
    double err = (*constr)->error();
    if (err * err < convergenceRedundant)
        redundant.insert(*constr);
}
resetToReference();
```

冗余约束集被存储，并将在initSolution过程中的其他函数中使用。

然而，关于冗余和冲突约束的信息被分开存储，以便可由求解器接口和Sketcher检索，以通知用户。此信息与上述冗余和冲突信息不同，因为：

- 标签等于零的冲突约束不报告
- 与不包含其他非冗余求解器约束的标签相关联的冗余求解器约束不报告，即它们源自部分冗余的Sketcher约束。理由是，告诉用户他无法移除的约束是部分冗余没有意义。用户对此无能为力。

### 5.4.2 划分为解耦组件

诊断过程确定冗余约束，这些约束从约束系统中移除，以便将系统划分为解耦组件。这包括源自部分冗余Sketcher约束的求解器约束。

结果是，如果Sketcher约束是部分冗余的，冗余部分会自动移除。如果Sketcher约束是完全冗余的，则会报告。已经多次询问为什么不让求解器完全忽略冗余约束。答案是消除Sketcher约束有助于防止更复杂的问题，并帮助用户拥有更清晰的Sketch。

解耦为组件使用图来执行。首先为每个参数和每个约束添加顶点。然后，对于每个约束，遍历相邻参数并创建连接相应约束顶点与给定参数在参数向量中的索引的边。

这使得能够确定存在多少组件（不连通的顶点）。

### 5.4.3 等式约束的识别和参数缩减

参数缩减通过缩减消除约束来执行。消除的约束存储在：

```cpp
std::set<Constraint *> reducedConstrs;
```

这些参数缩减被计算并存储在称为reductionmaps的映射向量中，为每个组件存储一个reductionmap。

```cpp
std::vector<std::map<double *, double *>> reductionmaps;
```

通过迭代求解器约束并为那些等式约束计算缩减映射，保留第一个参数，同时缩减第二个参数。这意味着第二个参数指向第一个参数。为每个组件存储此缩减参数的关联。

缩减后的参数和约束分别存储在clists和plists中。这包括每个组件的所有非缩减约束。

### 5.4.4 子系统组织

此步骤使用缩减后的参数和约束信息以及reductionmaps，将结果参数和约束组织到子系统中。

如前所述，要解决的实际子系统存储在GCS::SubSystem类型的向量中。

```cpp
std::vector<SubSystem *> subSystems, subSystemsAux;
```

这些向量首先被清除，并释放任何先前分配的内存。然后迭代每个解耦组件的缩减约束。为每个组件创建两个子系统，一个存储到subSystems中，另一个存储到subSystemsAux中。前者包含由非负标签的缩减约束形成的子系统，而后者包含由负标签的缩减约束形成的子系统。如前所述，负标签的约束源自几何图形的程序化移动。

## 5.5 求解

在求解器级别有一个通用求解函数。有三种特定求解函数用于特殊情况，并由通用求解函数使用。有四种算法实现修改的牛顿方法以执行实际求解。

### 5.5.1 参数求解

第一个特定求解函数接受要解决的参数。它为这些参数初始化解并使用通用求解函数求解系统。因此，特定部分是直接从参数进行特殊初始化。FreeCAD当前不使用此函数。

```cpp
int System::solve(VEC_pD &params, bool isFine, Algorithm alg, bool isRedundantsolving)
```

### 5.5.2 单个子系统求解

第二个特定求解函数接受一个没有优先级约束的子系统，即没有任何负标签的约束。此函数也是用于在诊断期间确定约束是冗余还是冲突的冗余求解函数（见5.4节）。实际函数是：

```cpp
int System::solve(SubSystem *subsys, bool isFine, Algorithm alg, bool isRedundantsolving)
```

求解单个子系统是最频繁的求解操作，仅依赖于为单个子系统求解实现的求解算法的实际实现。目前，这些算法是：

- Broyden-Fletcher-Goldfarb-Shanno（下文中为BFGS）
- Levenberg-Marquardt（下文中为LM）
- DogLeg（下文中为DL）

实现这些算法的函数是：

```cpp
int System::solve_BFGS(SubSystem *subsys, bool /*isFine*/, bool isRedundantsolving)
int System::solve_LM(SubSystem* subsys, bool isRedundantsolving)
int System::solve_DL(SubSystem* subsys, bool isRedundantsolving)
```

### 5.5.3 两个子系统求解

第三个特定求解函数实际上是算法的实现。这是因为只有一个算法用于同时求解两个子系统。该算法给予一个子系统优先于另一个子系统。这是用于几何图形程序化移动操作（如拖动）的算法。函数是：

```cpp
int System::solve(SubSystem *subsysA, SubSystem *subsysB, bool /*isFine*/, bool isRedundantsolving)
```

### 5.5.4 通用求解

求解器接口为执行求解操作而调用的函数（如4.4节所见）是：

```cpp
int System::solve(bool isFine, Algorithm alg, bool isRedundantsolving)
```

求解器级别的求解操作仅在系统已初始化时进行，即要解决的子系统已创建。

不同的解耦组件（即解耦子系统）分别求解。对于每个解耦组件，可能出现三种情况：

- 仅存在subSystems
- 仅存在subSystemsAux
- 同时存在subSystems和subSystemsAux

对于每个解耦组件中的每种情况，执行以下求解函数之一：

```cpp
int System::solve(SubSystem *subsys, bool isFine, Algorithm alg, bool isRedundantsolving)
int System::solve(SubSystem *subsysA, SubSystem *subsysB, bool /*isFine*/, bool isRedundantsolving)
```

解决所有组件后，如果所有求解都返回GCS::Success，则迭代所有约束并检查误差。如果任何单个约束的误差大于配置的收敛值，则将结果更改为GCS::Converged，指示求解器确实收敛了，但收敛未导致约束在所需误差级别上被强制执行。

如明显所示，当提供两个系统时，使用优先级求解算法执行两个子系统求解，而当提供一个系统时，使用作为参数提供的算法执行单个子系统求解。值得注意的是，当子系统同时包含正标签和负标签的约束时，Algorithm参数取何值无关紧要，执行特定的两个系统算法。此参数仅在提供单个子系统时才被考虑。

---

*本翻译文档基于FreeCAD Sketcher Solver Architecture.pdf, Abdullah Tahiri, December 2018*
