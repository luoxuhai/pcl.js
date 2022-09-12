/**
 * Copyright (c) Facebook, Inc. and its affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

import React, {
  type ComponentProps,
  type ReactElement,
  type ReactNode,
  isValidElement,
  useRef,
  useEffect,
} from 'react';
import { useHistory } from '@docusaurus/router';
import styles from './styles.module.css';

interface Props {
  readonly children: ReactElement<ComponentProps<'table'>>;
  readonly name?: string;
}

// ReactNode equivalent of HTMLElement#innerText
function getText(node: ReactElement): string {
  let curNode: ReactNode = node;
  while (isValidElement(curNode)) {
    [curNode] = React.Children.toArray(curNode.props.children);
  }
  return curNode as string;
}

function APITableRow(
  {
    name,
    children,
  }: { name: string | undefined; children: ReactElement<ComponentProps<'tr'>> },
  ref: React.ForwardedRef<HTMLTableRowElement>,
) {
  const entryName = getText(children);
  const id = name ? `${name}-${entryName}` : entryName;
  const anchor = `#${id}`;
  const history = useHistory();
  return (
    <tr
      id={id}
      tabIndex={0}
      ref={history.location.hash === anchor ? ref : undefined}
      onClick={() => {
        history.push(anchor);
      }}
      onKeyDown={(e: React.KeyboardEvent) => {
        if (e.key === 'Enter') {
          history.push(anchor);
        }
      }}
    >
      {children.props.children}
    </tr>
  );
}

const APITableRowComp = React.forwardRef(APITableRow);

/*
 * Note: this is not a quite robust component since it makes a lot of
 * assumptions about how the children looks; however, those assumptions
 * should be generally correct in the MDX context.
 */
export default function APITable({ children, name }: Props): JSX.Element {
  const [thead, tbody] = React.Children.toArray(children.props.children) as [
    ReactElement<{ children: ReactElement[] }>,
    ReactElement<{ children: ReactElement[] }>,
  ];
  const highlightedRow = useRef<HTMLTableRowElement>(null);
  useEffect(() => {
    highlightedRow.current?.focus();
  }, [highlightedRow]);
  const rows = React.Children.map(
    tbody.props.children,
    (row: ReactElement<ComponentProps<'tr'>>) => (
      <APITableRowComp name={name} ref={highlightedRow}>
        {row}
      </APITableRowComp>
    ),
  );

  return (
    <table className={styles.apiTable}>
      {thead}
      <tbody>{rows}</tbody>
    </table>
  );
}
