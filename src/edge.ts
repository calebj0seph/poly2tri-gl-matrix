/*
 * Poly2Tri Copyright (c) 2009-2018, Poly2Tri Contributors
 * https://github.com/jhasse/poly2tri
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * * Neither the name of Poly2Tri nor the names of its contributors may be
 *   used to endorse or promote products derived from this software without specific
 *   prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import { vec2 } from 'gl-matrix';

export interface Point {
  position: vec2;
  index: number;
  // The edges this point constitutes an upper ending point of
  edgeList: Edge[];
}

// Construct a point using coordinates without any edges
export function makePoint(position: vec2, index: number = -1): Point {
  return {
    position,
    index,
    edgeList: [],
  };
}

export function pointsEqual(a: Point, b: Point) {
  return a === b || vec2.equals(a.position, b.position);
}

export interface Edge {
  p: Point;
  q: Point;
}

// Represents a simple polygon's edge
export function makeEdge(p1: Point, p2: Point): Edge {
  let p = p1;
  let q = p2;

  if (p1.position[1] > p2.position[1]) {
    q = p1;
    p = p2;
  } else if (p1.position[1] === p2.position[1]) {
    if (p1.position[0] > p2.position[0]) {
      q = p1;
      p = p2;
    } else if (p1.position[0] === p2.position[0]) {
      // Repeat points
      throw new Error('makeEdge: p1 == p2');
    }
  }

  const edge: Edge = {
    p,
    q,
  };

  q.edgeList.push(edge);

  return edge;
}
