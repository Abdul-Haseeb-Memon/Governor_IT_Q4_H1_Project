/**
 * DOM Utilities
 * Functions for safely traversing and manipulating DOM elements
 */

/**
 * Extract text nodes from a given element while preserving structure
 * @param {Element} element - The DOM element to extract text from
 * @returns {Array} Array of text node objects with position and content
 */
export function extractTextNodes(element) {
  if (!element || !element.nodeType) {
    return [];
  }

  const textNodes = [];
  const walker = document.createTreeWalker(
    element,
    NodeFilter.SHOW_TEXT,
    {
      acceptNode: function(currentNode) {
        // Skip empty text nodes (whitespace-only)
        if (!currentNode.textContent.trim()) {
          return NodeFilter.FILTER_REJECT;
        }

        // Check if the parent element should be excluded
        if (shouldExcludeElement(currentNode.parentElement)) {
          return NodeFilter.FILTER_REJECT;
        }

        return NodeFilter.FILTER_ACCEPT;
      }
    }
  );

  let currentNode;
  while (currentNode = walker.nextNode()) {
    textNodes.push({
      node: currentNode,
      text: currentNode.textContent,
      parentElement: currentNode.parentElement,
      originalText: currentNode.textContent
    });
  }

  return textNodes;
}


/**
 * Check if an element should be excluded from translation
 * @param {Element} element - The element to check
 * @returns {boolean} - True if element should be excluded
 */
export function shouldExcludeElement(element) {
  if (!element) return false;

  // Tags to exclude
  const excludedTags = [
    'CODE', 'PRE', 'SCRIPT', 'STYLE', 'NOSCRIPT',
    'TEXTAREA', 'INPUT', 'SELECT', 'OPTION',
    'MATH', 'SVG', 'FORM', 'IFRAME', 'VIDEO',
    'AUDIO', 'CANVAS', 'OBJECT', 'EMBED', 'TT'
  ];

  // Check if element tag is in excluded list
  if (excludedTags.includes(element.tagName)) {
    return true;
  }

  // Check if element has CSS classes that indicate code or technical content
  const excludedClasses = [
    'code', 'code-block', 'code-block-container',
    'code-block-content', 'code-editor', 'terminal',
    'command', 'terminal', 'cli', 'shell', 'bash',
    'json', 'xml', 'yaml', 'toml', 'ini', 'config',
    'formula', 'math', 'equation', 'latex', 'language-text',
    'hljs', 'token', 'keyword', 'builtin', 'string', 'number',
    'comment', 'literal', 'operator', 'punctuation'
  ];

  if (element.className) {
    const elementClasses = element.className.toLowerCase().split(/\s+/);
    const hasExcludedClass = elementClasses.some(className =>
      excludedClasses.some(excludedClass =>
        className.includes(excludedClass)
      )
    );

    if (hasExcludedClass) {
      return true;
    }
  }

  // Check if element has data attributes indicating exclusion
  if (element.dataset && element.dataset.translate === 'false') {
    return true;
  }

  // Check for specific attributes that suggest technical content
  if (element.hasAttribute('data-lang') ||
      element.hasAttribute('data-language') ||
      element.hasAttribute('data-props') ||
      element.getAttribute('role') === 'code' ||
      element.getAttribute('aria-label')?.toLowerCase().includes('code')) {
    return true;
  }

  return false;
}


/**
 * Get all text content from an element excluding certain elements
 * @param {Element} element - The element to extract text from
 * @returns {string} - Combined text content
 */
export function getElementTextContent(element) {
  const walker = document.createTreeWalker(
    element,
    NodeFilter.SHOW_TEXT,
    {
      acceptNode: function(node) {
        if (!node.textContent.trim()) {
          return NodeFilter.FILTER_REJECT;
        }

        if (shouldExcludeElement(node.parentElement)) {
          return NodeFilter.FILTER_REJECT;
        }

        return NodeFilter.FILTER_ACCEPT;
      }
    }
  );

  const texts = [];
  let node;
  while (node = walker.nextNode()) {
    texts.push(node.textContent);
  }

  return texts.join(' ');
}