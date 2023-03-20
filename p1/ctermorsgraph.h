#ifndef CTERMORSGRAPH_H
#define CTERMORSGRAPH_H

//类CTerMorsGraph，用于描述ter Mors方法中提及的free time window graph
//在SequencePlan_StaticGraph函数中调用

class CTerMorsGraph
{
public:
    CTerMorsGraph();
    void UpdateGraph();//每次为新的对象规划路径前调用；根据当前各区域的时间窗构建或更新TerMorsGraph

};

#endif // CTERMORSGRAPH_H
