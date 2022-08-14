import React from 'react';
import Layout from '@theme/Layout';
import { getQueryString } from '../../utils/common';

export default function ExampleDetail() {
  const codeUrl = getQueryString('code_url');
  
  return (
    <Layout>
      <div
        className="container"
        style={{
          marginTop: 10,
        }}
      >
        {codeUrl ? (
          <iframe
            src={codeUrl}
            style={{
              width: '100%',
              height: 'calc(100vh - 60px - 20px)',
              borderRadius: 4,
            }}
          />
        ) : (
          <h2>Page Not Found</h2>
        )}
      </div>
    </Layout>
  );
}
