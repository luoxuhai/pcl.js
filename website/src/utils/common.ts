import { translate } from '@docusaurus/Translate';

export function getQueryString(name) {
  let reg = new RegExp('(^|&)' + name + '=([^&]*)(&|$)', 'i');
  if (typeof window !== 'undefined') {
    let r = window.location.search.substr(1).match(reg);
    if (r != null) {
      return decodeURIComponent(r[2]);
    }
  }

  return null;
}

export function translateMsg(message: string) {
  return translate({
    message,
  });
}
