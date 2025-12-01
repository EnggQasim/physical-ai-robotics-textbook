import React, { useState, useCallback, useRef, useMemo, useEffect } from 'react';
import ReactFlow, {
  Node,
  Edge,
  Controls,
  MiniMap,
  Background,
  BackgroundVariant,
  useNodesState,
  useEdgesState,
  NodeTypes,
  getNodesBounds,
  getViewportForBounds,
} from 'reactflow';
import { toPng } from 'html-to-image';
import 'reactflow/dist/style.css';

import MindMapNode from './MindMapNode';
import { getLayoutedElements } from './layout';
import styles from './styles.module.css';

interface MindMapViewerProps {
  chapterId: string;
  chapterTitle: string;
  chapterContent: string;
  onNavigateToSection?: (anchor: string) => void;
}

interface MindMapNodeData {
  id: string;
  label: string;
  description: string;
  level: number;
  parent_id: string | null;
  content_anchor?: string;
}

interface MindMapEdgeData {
  id: string;
  source: string;
  target: string;
  type?: string;
}

interface MindMapData {
  chapter_id: string;
  title: string;
  central_topic: MindMapNodeData;
  nodes: MindMapNodeData[];
  edges: MindMapEdgeData[];
  node_count: number;
  max_depth: number;
  generated_at: string;
}

interface MindMapResponse {
  success: boolean;
  mindmap?: MindMapData;
  cached: boolean;
  error?: string;
}

const API_URL = process.env.NODE_ENV === 'development'
  ? 'http://localhost:8000'
  : 'https://mqasim077-physical-ai-textbook-api.hf.space';

// Define custom node types outside component to prevent re-creation
const nodeTypes: NodeTypes = {
  mindmapNode: MindMapNode,
};

function convertToReactFlowNodes(mindmap: MindMapData): { nodes: Node[]; edges: Edge[] } {
  // Collect all nodes including central topic
  const allNodes: MindMapNodeData[] = [mindmap.central_topic, ...mindmap.nodes];

  // Count children for each node
  const childCounts: Record<string, number> = {};
  allNodes.forEach(node => {
    if (node.parent_id) {
      childCounts[node.parent_id] = (childCounts[node.parent_id] || 0) + 1;
    }
  });

  // Convert to React Flow format
  const nodes: Node[] = allNodes.map((node) => ({
    id: node.id,
    type: 'mindmapNode',
    data: {
      label: node.label,
      description: node.description,
      level: node.level,
      hasChildren: (childCounts[node.id] || 0) > 0,
      contentAnchor: node.content_anchor,
    },
    position: { x: 0, y: 0 }, // Will be set by layout
  }));

  const edges: Edge[] = mindmap.edges.map((edge) => ({
    id: edge.id,
    source: edge.source,
    target: edge.target,
    type: 'default', // Bezier curves for organic look
    animated: false,
    style: { stroke: '#9ca3af', strokeWidth: 3 },
  }));

  return getLayoutedElements(nodes, edges, 'TB');
}

export default function MindMapViewer({
  chapterId,
  chapterTitle,
  chapterContent,
  onNavigateToSection,
}: MindMapViewerProps): JSX.Element {
  const [nodes, setNodes, onNodesChange] = useNodesState([]);
  const [edges, setEdges, onEdgesChange] = useEdgesState([]);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [isCached, setIsCached] = useState(false);
  const [hasLoaded, setHasLoaded] = useState(false);
  const [isExporting, setIsExporting] = useState(false);
  const [mindmapInfo, setMindmapInfo] = useState<{ nodeCount: number; generatedAt: string } | null>(null);

  const reactFlowWrapper = useRef<HTMLDivElement>(null);

  const fetchMindMap = useCallback(async (forceRegenerate: boolean = false) => {
    setIsLoading(true);
    setError(null);

    try {
      const response = await fetch(`${API_URL}/api/mindmap/generate`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          chapter_id: chapterId,
          chapter_content: chapterContent,
          chapter_title: chapterTitle,
          force_regenerate: forceRegenerate,
        }),
      });

      if (!response.ok) {
        throw new Error(`Failed to generate mind map: ${response.statusText}`);
      }

      const data: MindMapResponse = await response.json();

      if (data.success && data.mindmap) {
        const { nodes: layoutedNodes, edges: layoutedEdges } = convertToReactFlowNodes(data.mindmap);
        setNodes(layoutedNodes);
        setEdges(layoutedEdges);
        setIsCached(data.cached);
        setMindmapInfo({
          nodeCount: data.mindmap.node_count,
          generatedAt: data.mindmap.generated_at,
        });
        setHasLoaded(true);
      } else {
        setError(data.error || 'Failed to generate mind map');
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : 'An error occurred');
    } finally {
      setIsLoading(false);
    }
  }, [chapterId, chapterContent, chapterTitle, setNodes, setEdges]);

  // Auto-fetch on mount if not loaded
  useEffect(() => {
    if (!hasLoaded && !isLoading && chapterContent.length > 100) {
      fetchMindMap();
    }
  }, [hasLoaded, isLoading, chapterContent, fetchMindMap]);

  const handleRegenerate = () => {
    fetchMindMap(true);
  };

  const handleExport = async () => {
    if (!reactFlowWrapper.current || nodes.length === 0) return;

    setIsExporting(true);
    try {
      // Calculate bounds of all nodes
      const nodesBounds = getNodesBounds(nodes);
      const imageWidth = 1920;
      const imageHeight = 1080;

      // Get viewport that fits all nodes
      const viewport = getViewportForBounds(
        nodesBounds,
        imageWidth,
        imageHeight,
        0.5,
        2,
        0.2
      );

      // Find the viewport element and apply the calculated transform
      const viewportElement = reactFlowWrapper.current.querySelector('.react-flow__viewport') as HTMLElement;
      if (!viewportElement) {
        throw new Error('Could not find mind map viewport');
      }

      // Store original transform
      const originalTransform = viewportElement.style.transform;

      // Apply the calculated viewport transform for export
      viewportElement.style.transform = `translate(${viewport.x}px, ${viewport.y}px) scale(${viewport.zoom})`;

      const dataUrl = await toPng(viewportElement, {
        backgroundColor: '#ffffff',
        quality: 1.0,
        width: imageWidth,
        height: imageHeight,
        style: {
          width: `${imageWidth}px`,
          height: `${imageHeight}px`,
        },
      });

      // Restore original transform
      viewportElement.style.transform = originalTransform;

      const link = document.createElement('a');
      link.download = `mindmap-${chapterId}.png`;
      link.href = dataUrl;
      link.click();
    } catch (err) {
      console.error('Failed to export mind map:', err);
    } finally {
      setIsExporting(false);
    }
  };

  const handleNodeDoubleClick = useCallback((event: React.MouseEvent, node: Node) => {
    const anchor = node.data?.contentAnchor;
    if (anchor && onNavigateToSection) {
      onNavigateToSection(anchor);
    }
  }, [onNavigateToSection]);

  const proOptions = useMemo(() => ({ hideAttribution: true }), []);

  if (isLoading) {
    return (
      <div className={styles.container}>
        <div className={styles.loadingContainer}>
          <div className={styles.spinner} />
          <span className={styles.loadingText}>Generating Mind Map...</span>
          <span className={styles.loadingSubtext}>Analyzing chapter concepts and relationships</span>
        </div>
      </div>
    );
  }

  if (error) {
    return (
      <div className={styles.container}>
        <div className={styles.errorContainer}>
          <svg className={styles.errorIcon} width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
            <circle cx="12" cy="12" r="10" />
            <line x1="12" y1="8" x2="12" y2="12" />
            <line x1="12" y1="16" x2="12.01" y2="16" />
          </svg>
          <span className={styles.errorText}>{error}</span>
          <button className={styles.retryButton} onClick={() => fetchMindMap()}>
            Try Again
          </button>
        </div>
      </div>
    );
  }

  if (!hasLoaded || nodes.length === 0) {
    return (
      <div className={styles.container}>
        <div className={styles.emptyContainer}>
          <svg className={styles.emptyIcon} width="48" height="48" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5">
            <circle cx="12" cy="12" r="3" />
            <path d="M12 2v4" />
            <path d="M12 18v4" />
            <path d="M4.93 4.93l2.83 2.83" />
            <path d="M16.24 16.24l2.83 2.83" />
            <path d="M2 12h4" />
            <path d="M18 12h4" />
            <path d="M4.93 19.07l2.83-2.83" />
            <path d="M16.24 7.76l2.83-2.83" />
          </svg>
          <span className={styles.emptyText}>Generate an interactive mind map of this chapter</span>
          <button className={styles.generateButton} onClick={() => fetchMindMap()}>
            Generate Mind Map
          </button>
        </div>
      </div>
    );
  }

  return (
    <div className={styles.container}>
      <div className={styles.header}>
        <div className={styles.headerLeft}>
          <h3 className={styles.title}>Mind Map</h3>
          {isCached && (
            <span className={styles.cachedBadge}>
              <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <polyline points="20 6 9 17 4 12" />
              </svg>
              Cached
            </span>
          )}
          {mindmapInfo && (
            <span className={styles.nodeCountBadge}>
              {mindmapInfo.nodeCount} nodes
            </span>
          )}
        </div>
        <div className={styles.headerActions}>
          <button
            className={styles.exportButton}
            onClick={handleExport}
            disabled={isExporting}
            title="Export as PNG"
          >
            {isExporting ? (
              <div className={styles.smallSpinner} />
            ) : (
              <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4" />
                <polyline points="7 10 12 15 17 10" />
                <line x1="12" y1="15" x2="12" y2="3" />
              </svg>
            )}
            Export PNG
          </button>
          <button
            className={styles.regenerateButton}
            onClick={handleRegenerate}
            title="Regenerate mind map"
          >
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <path d="M21 2v6h-6" />
              <path d="M3 12a9 9 0 0 1 15-6.7L21 8" />
              <path d="M3 22v-6h6" />
              <path d="M21 12a9 9 0 0 1-15 6.7L3 16" />
            </svg>
            Regenerate
          </button>
        </div>
      </div>

      <div className={styles.flowContainer} ref={reactFlowWrapper}>
        <ReactFlow
          nodes={nodes}
          edges={edges}
          onNodesChange={onNodesChange}
          onEdgesChange={onEdgesChange}
          onNodeDoubleClick={handleNodeDoubleClick}
          nodeTypes={nodeTypes}
          fitView
          fitViewOptions={{ padding: 0.3 }}
          minZoom={0.3}
          maxZoom={2}
          proOptions={proOptions}
          defaultViewport={{ x: 0, y: 0, zoom: 0.8 }}
        >
          <Background variant={BackgroundVariant.Dots} gap={20} size={1} color="#e5e7eb" />
          <Controls showInteractive={false} />
          <MiniMap
            nodeColor={(node) => {
              // Use the node's assigned color for minimap
              return node.data?.color || '#76b900';
            }}
            maskColor="rgba(0, 0, 0, 0.08)"
            style={{ backgroundColor: '#fafafa' }}
          />
        </ReactFlow>
      </div>

      <div className={styles.footer}>
        <span className={styles.footerHint}>
          <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
            <circle cx="12" cy="12" r="10" />
            <path d="M12 16v-4" />
            <path d="M12 8h.01" />
          </svg>
          Hover over nodes for details. Use scroll to zoom, drag to pan.
        </span>
        {mindmapInfo && (
          <span className={styles.generatedAt}>
            Generated: {new Date(mindmapInfo.generatedAt).toLocaleDateString()}
          </span>
        )}
      </div>
    </div>
  );
}
