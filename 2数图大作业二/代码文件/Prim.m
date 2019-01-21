function [Trees] = Prim(Nodes,DictImg)
%% 求解邻接矩阵
NumNodes = length(Nodes);
[cpt_x, cpt_y, cpt_z] = ind2sub(size(DictImg), Nodes);
X1 = repmat(cpt_x,1,NumNodes);
X2 = repmat(cpt_x',NumNodes,1);
Y1 = repmat(cpt_y,1,NumNodes);
Y2 = repmat(cpt_y',NumNodes,1);
Z1 = repmat(cpt_z,1,NumNodes);
Z2 = repmat(cpt_z',NumNodes,1); 
Graph = ((X1-X2).^2 + (Y1-Y2).^2 +(Z1-Z2).^2).^(0.5);
%同一连通域，距离设置为0
M1 = repmat(DictImg(Nodes),1,NumNodes);
M2 = repmat(DictImg(Nodes)',NumNodes,1);
Mask = ((M1 - M2)~=0);
Graph = Graph .* Mask;
%% Prim算法
VisitedNode = 1;
NeighborNode = 2:NumNodes;
Trees = [];
while length(VisitedNode) ~= NumNodes
    CostCurr = Graph(VisitedNode,NeighborNode);
    [Costmin,index] = min(CostCurr(:));
    [SrcNode,DstNode] = ind2sub(size(CostCurr), index);
    SrcNode = VisitedNode(SrcNode(1));
    DstNode = NeighborNode(DstNode(1));
    Trees = [Trees;SrcNode,DstNode,Costmin];
    VisitedNode = [VisitedNode;DstNode];
    NeighborNode(find(NeighborNode==DstNode)) = [];
end

end

