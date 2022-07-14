import { vec2 } from 'gl-matrix';
import { makePoint } from './edge';
import { triangulate } from './sweep';
import { SweepContext } from './sweep-context';

/**
 * Triangulates the given polygon.
 *
 * @param points The points of the polygon.
 * @returns The indices of three points at a time which make up each triangle.
 */
export function triangulatePolygon(points: vec2[]) {
  const sweepContext = new SweepContext(points.map((p, i) => makePoint(p, i)));

  triangulate(sweepContext);
  const triangles = sweepContext.getTriangles();

  return triangles.flatMap((t) => {
    const indices = [
      t.getPoint(0).index,
      t.getPoint(1).index,
      t.getPoint(2).index,
    ];

    if (indices[0] === -1 || indices[1] === -1 || indices[2] === -1) {
      throw new Error('triangulatePolygon: missing index on point');
    }

    return indices;
  });
}
