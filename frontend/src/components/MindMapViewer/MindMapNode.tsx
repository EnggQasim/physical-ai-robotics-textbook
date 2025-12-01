import React, { memo, useState } from 'react';
import { Handle, Position, NodeProps } from 'reactflow';
import styles from './styles.module.css';

interface MindMapNodeData {
  label: string;
  description: string;
  level: number;
  hasChildren?: boolean;
  contentAnchor?: string;
  color?: string;
}

function MindMapNode({ data }: NodeProps<MindMapNodeData>) {
  const [showTooltip, setShowTooltip] = useState(false);

  // NotebookLM style: colored backgrounds based on branch
  const getNodeStyle = (): React.CSSProperties => {
    const baseStyle: React.CSSProperties = {
      padding: '10px 16px',
      borderRadius: data.level === 0 ? '50%' : '20px',
      minWidth: data.level === 0 ? '100px' : '80px',
      maxWidth: data.level === 0 ? '120px' : '150px',
      textAlign: 'center',
      cursor: 'pointer',
      transition: 'all 0.2s ease',
      boxShadow: '0 2px 8px rgba(0, 0, 0, 0.15)',
      border: 'none',
      position: 'relative',
    };

    if (data.level === 0) {
      // Central node - larger, circular, green
      return {
        ...baseStyle,
        background: 'linear-gradient(135deg, #76b900 0%, #5a9400 100%)',
        color: 'white',
        fontWeight: 700,
        fontSize: '13px',
        minHeight: '100px',
        minWidth: '100px',
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'center',
      };
    } else if (data.level === 1) {
      // Level 1 - colored pill shapes
      const color = data.color || '#4285f4';
      return {
        ...baseStyle,
        background: color,
        color: 'white',
        fontWeight: 600,
        fontSize: '12px',
      };
    } else {
      // Level 2+ - lighter colored pills with border
      const color = data.color || '#4285f4';
      return {
        ...baseStyle,
        background: `${color}20`, // 12% opacity
        color: color,
        fontWeight: 500,
        fontSize: '11px',
        border: `2px solid ${color}`,
      };
    }
  };

  return (
    <div
      style={getNodeStyle()}
      onMouseEnter={() => setShowTooltip(true)}
      onMouseLeave={() => setShowTooltip(false)}
    >
      {/* Handles for connections - invisible */}
      <Handle
        type="target"
        position={Position.Top}
        style={{
          background: 'transparent',
          border: 'none',
          width: 1,
          height: 1,
        }}
      />

      <div className={styles.nodeContent}>
        <span className={styles.nodeLabel}>{data.label}</span>
      </div>

      {/* Tooltip on hover */}
      {showTooltip && data.description && (
        <div className={styles.tooltip}>
          {data.description}
          {data.contentAnchor && (
            <span className={styles.tooltipHint}>Double-click to navigate</span>
          )}
        </div>
      )}

      <Handle
        type="source"
        position={Position.Bottom}
        style={{
          background: 'transparent',
          border: 'none',
          width: 1,
          height: 1,
        }}
      />
    </div>
  );
}

export default memo(MindMapNode);
