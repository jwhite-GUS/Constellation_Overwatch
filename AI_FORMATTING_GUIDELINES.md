# AI Formatting Guidelines for Constellation Overwatch SDK

<!-- DEVTEAM: This file establishes professional formatting standards for all AI-generated content -->

## Professional Documentation Standards

### General Principles
- **Professional Tone**: Maintain a serious, government-appropriate tone
- **No Decorative Elements**: Avoid emojis, unnecessary icons, or decorative characters
- **Clear Structure**: Use consistent heading hierarchy and formatting
- **Technical Focus**: Emphasize functionality and technical accuracy

### Formatting Rules

#### Headers and Sections
```markdown
# Primary Title
## Major Section
### Subsection
#### Detail Section
```

#### Lists and Bullets
Use standard markdown bullets without decorative elements:
```markdown
- Standard bullet point
- Another item
  - Sub-item
  - Sub-item
```

#### Status Indicators
Instead of emojis, use standard text indicators:
```markdown
- COMPLETE: Feature implementation finished
- IN PROGRESS: Currently under development
- PLANNED: Scheduled for future release
- DEPRECATED: No longer supported
```

#### Code and Technical Content
```markdown
**Code blocks**: Use proper syntax highlighting
`inline code`: Use backticks for inline code references
**File paths**: Use standard path notation without decorative elements
```

#### Professional Alternatives to Common Emojis
- ✅ → COMPLETE or [DONE]
- ❌ → INCOMPLETE or [PENDING]
- 🚀 → DEPLOYMENT or LAUNCH
- 📚 → DOCUMENTATION
- 🔧 → CONFIGURATION or TOOLS
- 🎯 → OBJECTIVES or GOALS
- 🌟 → FEATURED or IMPORTANT

## DEVTEAM Integration Notes

### File Header Comments
All files should include a DEVTEAM note at the top:
```markdown
<!-- DEVTEAM: [Brief description of file purpose and formatting requirements] -->
```

### Section Comments
Add DEVTEAM notes for major sections:
```markdown
<!-- DEVTEAM: This section covers [specific topic] - maintain professional tone -->
```

### Code Comments
For Python files:
```python
# DEVTEAM: This module handles [specific functionality]
# Follow professional coding standards and documentation
```

For shell scripts:
```bash
# DEVTEAM: This script performs [specific task]
# Use professional output messages without decorative elements
```

## Implementation Guidelines

### When Updating Documentation
1. Replace all emojis with professional text equivalents
2. Add appropriate DEVTEAM notes for context
3. Maintain technical accuracy and clarity
4. Use consistent formatting throughout

### When Writing Code
1. Include DEVTEAM notes for complex functions
2. Use professional variable and function names
3. Add comprehensive docstrings
4. Follow established coding standards

### When Creating New Files
1. Start with appropriate DEVTEAM note
2. Use professional headers and structure
3. Include clear purpose statements
4. Maintain consistency with existing files

## Quality Assurance

### Before Committing
- [ ] All emojis replaced with professional alternatives
- [ ] DEVTEAM notes added where appropriate
- [ ] Consistent formatting throughout
- [ ] Technical accuracy verified
- [ ] Professional tone maintained

### Review Checklist
- [ ] Headers use standard markdown format
- [ ] Lists use standard bullet points
- [ ] Code blocks properly formatted
- [ ] File paths and references clear
- [ ] Documentation complete and accurate
