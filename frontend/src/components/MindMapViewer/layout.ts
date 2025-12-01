import { Node, Edge } from 'reactflow';

const NODE_WIDTH = 160;
const NODE_HEIGHT = 50;

// NotebookLM-style color palette for branches
const BRANCH_COLORS = [
  '#4285f4', // Blue
  '#ea4335', // Red
  '#fbbc04', // Yellow
  '#34a853', // Green
  '#ff6d01', // Orange
  '#46bdc6', // Teal
  '#7baaf7', // Light Blue
  '#f07b72', // Coral
  '#a142f4', // Purple
  '#24c1e0', // Cyan
];

/**
 * Apply radial/organic layout for NotebookLM-style mind map
 * Central node in the middle, branches spread outward
 */
export function getLayoutedElements(
  nodes: Node[],
  edges: Edge[],
  direction: 'TB' | 'LR' = 'TB'
): { nodes: Node[]; edges: Edge[] } {
  if (nodes.length === 0) {
    return { nodes: [], edges: [] };
  }

  // Find the central node (level 0)
  const centralNode = nodes.find(n => n.data?.level === 0);
  const level1Nodes = nodes.filter(n => n.data?.level === 1);
  const level2Nodes = nodes.filter(n => n.data?.level === 2);

  // Build parent-child relationships
  const childrenMap: Record<string, string[]> = {};
  edges.forEach(edge => {
    if (!childrenMap[edge.source]) {
      childrenMap[edge.source] = [];
    }
    childrenMap[edge.source].push(edge.target);
  });

  // Assign colors to level 1 branches
  const nodeColors: Record<string, string> = {};
  level1Nodes.forEach((node, index) => {
    const color = BRANCH_COLORS[index % BRANCH_COLORS.length];
    nodeColors[node.id] = color;
    // Propagate color to children
    const children = childrenMap[node.id] || [];
    children.forEach(childId => {
      nodeColors[childId] = color;
    });
  });

  // Layout parameters
  const centerX = 400;
  const centerY = 300;
  const level1Radius = 220;
  const level2Radius = 400;

  const layoutedNodes: Node[] = [];

  // Position central node
  if (centralNode) {
    layoutedNodes.push({
      ...centralNode,
      position: { x: centerX - NODE_WIDTH / 2, y: centerY - NODE_HEIGHT / 2 },
      data: {
        ...centralNode.data,
        color: '#76b900', // NVIDIA green for central
      },
    });
  }

  // Position level 1 nodes in a circle around center
  const level1Count = level1Nodes.length;
  level1Nodes.forEach((node, index) => {
    const angle = (2 * Math.PI * index) / level1Count - Math.PI / 2; // Start from top
    const x = centerX + level1Radius * Math.cos(angle) - NODE_WIDTH / 2;
    const y = centerY + level1Radius * Math.sin(angle) - NODE_HEIGHT / 2;

    layoutedNodes.push({
      ...node,
      position: { x, y },
      data: {
        ...node.data,
        color: nodeColors[node.id],
      },
    });
  });

  // Position level 2 nodes around their parents
  level1Nodes.forEach((parentNode, parentIndex) => {
    const children = level2Nodes.filter(n => {
      const parentEdge = edges.find(e => e.target === n.id);
      return parentEdge?.source === parentNode.id;
    });

    if (children.length === 0) return;

    // Parent's angle
    const parentAngle = (2 * Math.PI * parentIndex) / level1Count - Math.PI / 2;

    // Spread children in an arc extending from parent
    const arcSpread = Math.PI / 3; // 60 degree arc
    const startAngle = parentAngle - arcSpread / 2;

    children.forEach((child, childIndex) => {
      const childAngle = startAngle + (arcSpread * (childIndex + 0.5)) / Math.max(children.length, 1);
      const x = centerX + level2Radius * Math.cos(childAngle) - NODE_WIDTH / 2;
      const y = centerY + level2Radius * Math.sin(childAngle) - NODE_HEIGHT / 2;

      layoutedNodes.push({
        ...child,
        position: { x, y },
        data: {
          ...child.data,
          color: nodeColors[child.id],
        },
      });
    });
  });

  // Update edge colors to match their source node
  const coloredEdges = edges.map(edge => {
    const sourceNode = layoutedNodes.find(n => n.id === edge.source);
    const color = sourceNode?.data?.color || '#9ca3af';
    return {
      ...edge,
      style: {
        ...edge.style,
        stroke: color,
        strokeWidth: 3,
      },
      animated: false,
    };
  });

  return {
    nodes: layoutedNodes,
    edges: coloredEdges,
  };
}

export { NODE_WIDTH, NODE_HEIGHT, BRANCH_COLORS };
