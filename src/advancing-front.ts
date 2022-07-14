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

import { Point, pointsEqual } from './edge';
import { Triangle } from './triangle';

export interface Node {
  next: Node | null;
  point: Point;
  prev: Node | null;
  triangle: Triangle | null;
  value: number;
}

export function makeNode(p: Point, t: Triangle | null = null): Node {
  return {
    next: null,
    point: p,
    prev: null,
    triangle: t,
    value: p.position[0],
  };
}

export class AdvancingFront {
  public head: Node | null;
  public search: Node | null;
  public tail: Node | null;

  public constructor(head: Node, tail: Node) {
    this.head = head;
    this.tail = tail;
    this.search = head;
  }

  private findSearchNode(x: number): Node | null {
    // TODO: implement BST index
    return this.search;
  }

  public locateNode(x: number): Node | null {
    let node: Node | null = this.search;

    if (node === null) {
      return null;
    }

    if (x < node.value) {
      while ((node = node.prev) !== null)
        if (x >= node.value) {
          this.search = node;
          return node;
        }
    } else {
      while ((node = node.next) !== null)
        if (x < node.value) {
          this.search = node.prev;
          return node.prev;
        }
    }

    return null;
  }

  public locatePoint(point: Point): Node | null {
    const px = point.position[0];
    let node: Node | null = this.findSearchNode(px);

    if (node === null) {
      return null;
    }

    const nx = node.point.position[0];
    if (px === nx) {
      if (!pointsEqual(point, node.point)) {
        // We might have two nodes with same x value for a short time
        if (node.prev !== null && pointsEqual(point, node.prev.point))
          node = node.prev;
        else if (node.next !== null && pointsEqual(point, node.next.point))
          node = node.next;
        else throw new Error('AdvancingFront::locatePoint: invalid point');
      }
    } else if (px < nx) {
      while ((node = node.prev) !== null)
        if (pointsEqual(point, node.point)) break;
    } else {
      while ((node = node.next) !== null)
        if (pointsEqual(point, node.point)) break;
    }

    if (node !== null) this.search = node;

    return node;
  }
}
